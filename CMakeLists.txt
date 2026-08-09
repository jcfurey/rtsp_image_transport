############################################################################
#
# rtsp_image_transport
# Copyright © 2021 Fraunhofer FKIE
# Author: Timo Röhling
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
############################################################################
cmake_minimum_required(VERSION 3.13...3.28)
project(
    rtsp_image_transport
    VERSION 2.0.0
    LANGUAGES CXX
)

# ament executes package.xml helpers with the interpreter selected by CMake.
# cam-ripper has an unrelated ~/.local Python 3.11 ahead of the ROS Lyrical
# system interpreter, but that environment does not contain catkin_pkg. Honour
# an explicit caller override; otherwise select the Ubuntu system Python that
# owns the ROS Python modules.
if(NOT Python3_EXECUTABLE)
    find_program(
        ROS_SYSTEM_PYTHON
        NAMES python3
        PATHS /usr/bin
        NO_DEFAULT_PATH
        REQUIRED
    )
    set(Python3_EXECUTABLE "${ROS_SYSTEM_PYTHON}" CACHE FILEPATH "Python interpreter used by ROS" FORCE)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

include(GenerateExportHeader)
include(CheckCXXSourceCompiles)

# Dependencies
#
# ament_auto derives the ROS dependencies from package.xml, so the list lives
# in exactly one place. The two non-ament dependencies still need naming here:
# live555_vendor exports its CMake package as "live555", and ffmpeg is a plain
# system library found through pkg-config.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(live555 REQUIRED)
find_package(PkgConfig REQUIRED)

# Need at least FFmpeg 3
pkg_check_modules(
    ffmpeg
    REQUIRED
    IMPORTED_TARGET
    libavcodec>=57
    libavformat>=57
    libavutil>=55
    libswscale>=4
)

# Check compatibility of found libraries
set(CMAKE_REQUIRED_LIBRARIES "PkgConfig::ffmpeg")
check_cxx_source_compiles(
    [==[
extern "C"
{
#include <libavcodec/avcodec.h>
}

int main()
{
    AVHWFramesContext* c1 = nullptr;
    AVHWDeviceContext* c2 = nullptr;
    AVPixelFormat f = AV_PIX_FMT_VAAPI;
}
]==]
    FFMPEG_HAS_HWFRAME_SUPPORT
)

# Multi-threaded colour space conversion. Only sws_scale_frame() distributes the
# work over the worker threads, and neither it nor the manual context setup is
# available in every supported libswscale release.
check_cxx_source_compiles(
    [==[
extern "C"
{
#include <libavutil/buffer.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

int main()
{
    SwsContext* c = sws_alloc_context();
    av_opt_set_int(c, "threads", 2, 0);
    if (sws_init_context(c, nullptr, nullptr) < 0)
        return 1;
    AVFrame* f = av_frame_alloc();
    f->buf[0] = av_buffer_create(nullptr, 0, nullptr, nullptr, 0);
    return sws_scale_frame(c, f, f);
}
]==]
    FFMPEG_HAS_SWS_THREADS
)

# Older versions of live555 had a bug that prevented the proper inclusion of the
# H.265 classes
set(CMAKE_REQUIRED_LIBRARIES "live555::liveMedia")
check_cxx_source_compiles(
    [==[
#include <liveMedia.hh>

int main()
{
    H265VideoRTPSink* sink = nullptr;
    H265VideoStreamDiscreteFramer* framer = nullptr;
}
]==]
    LIVE555_HAS_H265_SUPPORT
)

check_cxx_source_compiles(
    [==[
#include <liveMedia.hh>

int main()
{
    VP8VideoRTPSink* sink1 = nullptr;
    VP9VideoRTPSink* sink2 = nullptr;
}
]==]
    LIVE555_HAS_VPX_SUPPORT
)
check_cxx_source_compiles(
    [==[
#include <liveMedia.hh>

int main()
{
    AV1VideoRTPSink* sink = nullptr;
}
]==]
    LIVE555_HAS_AV1_SUPPORT
)

unset(CMAKE_REQUIRED_LIBRARIES)

# rclcpp version
math(
    EXPR
    computed_rclcpp_version
    "65536 * ${rclcpp_VERSION_MAJOR} + 256 * ${rclcpp_VERSION_MINOR} + ${rclcpp_VERSION_PATCH}"
)

# image_transport version. From 6.4 onwards the plugin entry points take node
# interfaces instead of a node, and the old ones are no longer called.
math(
    EXPR
    computed_image_transport_version
    "65536 * ${image_transport_VERSION_MAJOR} + 256 * ${image_transport_VERSION_MINOR} + ${image_transport_VERSION_PATCH}"
)

# Targets

# Everything except the two image_transport plugins lives in a static core
# library, so that the unit tests can link against it. It stays a plain target
# rather than an ament_auto one, because it is an implementation detail that
# should not end up in this package's exported interface.
add_library(
    ${PROJECT_NAME}_core STATIC
    src/event_loop.cpp
    src/frame_data.cpp
    src/frame_extractor.cpp
    src/frame_injector.cpp
    src/graph_monitor.cpp
    src/host_override.cpp
    src/init.cpp
    src/stream_clock.cpp
    src/stream_client.cpp
    src/stream_decoder.cpp
    src/stream_encoder.cpp
    src/stream_server.cpp
    src/video_codec.cpp
)
set_target_properties(
    ${PROJECT_NAME}_core
    PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET "hidden"
               VISIBILITY_INLINES_HIDDEN ON
)
target_include_directories(
    ${PROJECT_NAME}_core PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
                                ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_20)
target_compile_definitions(
    ${PROJECT_NAME}_core
    PUBLIC $<$<BOOL:${LIVE555_HAS_H265_SUPPORT}>:LIVE555_HAS_H265_SUPPORT>
           $<$<BOOL:${LIVE555_HAS_VPX_SUPPORT}>:LIVE555_HAS_VPX_SUPPORT>
           $<$<BOOL:${LIVE555_HAS_AV1_SUPPORT}>:LIVE555_HAS_AV1_SUPPORT>
           $<$<BOOL:${FFMPEG_HAS_HWFRAME_SUPPORT}>:FFMPEG_HAS_HWFRAME_SUPPORT>
           $<$<BOOL:${FFMPEG_HAS_SWS_THREADS}>:FFMPEG_HAS_SWS_THREADS>
           CURRENT_RCLCPP_VERSION=${computed_rclcpp_version}
           CURRENT_IMAGE_TRANSPORT_VERSION=${computed_image_transport_version}
)
# Named explicitly rather than through ament_target_dependencies(), which was
# removed in Lyrical. Message packages are linked through their _TARGETS
# variable: the sensor_msgs::sensor_msgs style alias no longer exists in Kilted
# and later.
target_link_libraries(
    ${PROJECT_NAME}_core PUBLIC live555::liveMedia PkgConfig::ffmpeg
                                rclcpp::rclcpp ${sensor_msgs_TARGETS}
                                ${std_msgs_TARGETS}
)

# Added before the plugin module so that ament_auto does not try to link the
# module into them.
ament_auto_add_executable(publish_rtsp_stream src/publish_rtsp_stream.cpp)
target_compile_definitions(
    publish_rtsp_stream PRIVATE "ROS_PACKAGE_NAME=\"${PROJECT_NAME}\""
)
# Shares the topic name helpers with the plugins, which are C++20. Without this
# the executable was the one target still built at the compiler default.
target_compile_features(publish_rtsp_stream PRIVATE cxx_std_20)

ament_auto_add_executable(
    rtsp_camera_proxy src/host_override.cpp src/rtsp_camera_proxy.cpp
)
target_compile_features(rtsp_camera_proxy PRIVATE cxx_std_20)
target_link_libraries(rtsp_camera_proxy live555::liveMedia)

# The image_transport plugins, loaded through pluginlib and never linked
# against directly. Deliberately not an ament_auto target: CMake refuses to
# link a MODULE library into anything, so registering it would break every
# ament_auto target declared after it, and exporting it as a linkable library
# would be wrong in any case.
add_library(
    ${PROJECT_NAME} MODULE src/publisher_plugin.cpp src/subscriber_plugin.cpp
)
generate_export_header(${PROJECT_NAME})
set_target_properties(
    ${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden"
                               VISIBILITY_INLINES_HIDDEN ON
)
target_link_libraries(
    ${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_core
                            image_transport::image_transport
                            pluginlib::pluginlib
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
    ament_auto_find_test_dependencies()

    set(unit_tests
        test_video_codec
        test_event_loop
        test_stream_clock
        test_time_handling
        test_frame_data
        test_stream_decoder
        test_stream_encoder
        test_roundtrip
        test_host_override
        test_graph_monitor
        test_topic_parameter
        test_codec_tables
        test_frame_extractor
        test_reconnect_policy
        test_ffmpeg_logging
    )
    # Serve a real RTSP session; the transport tests additionally drive the
    # image_transport plugins through pluginlib.
    list(APPEND unit_tests test_rtsp_loopback test_sim_time)

    # colcon runs the test binaries in parallel and video coding is not cheap,
    # so the timeout is generous enough that a loaded build server does not
    # report a slow run as a failure. ament_auto_add_gtest() links this
    # package's dependencies with the plain target_link_libraries() signature,
    # so the core has to be added plainly too.
    foreach(test_name IN LISTS unit_tests)
        ament_auto_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 180)
        if(TARGET ${test_name})
            target_link_libraries(${test_name} ${PROJECT_NAME}_core)
            set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=157")
        endif()
    endforeach()

    # Runs the two command line programs as child processes, so it needs to
    # know where they were built and must not start before they exist. Its own
    # ROS domain: it waits on latched topics by name, and the loopback tests
    # publish similar ones alongside it.
    ament_auto_add_gtest(test_executables test/test_executables.cpp TIMEOUT 300)
    if(TARGET test_executables)
        target_link_libraries(test_executables ${PROJECT_NAME}_core)
        target_compile_definitions(
            test_executables PRIVATE "RTSP_BINARY_DIR=\"${CMAKE_CURRENT_BINARY_DIR}\""
        )
        add_dependencies(test_executables publish_rtsp_stream rtsp_camera_proxy)
        set_tests_properties(
            test_executables PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=158"
        )
    endif()
endif()

# Ament package handling. ament_auto_package() installs the executables to
# lib/${PROJECT_NAME}, which is where ros2 run looks for them.
pluginlib_export_plugin_description_file(image_transport plugin.xml)
ament_auto_package()
