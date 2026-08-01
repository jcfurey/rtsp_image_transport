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

include(GenerateExportHeader)
include(CheckCXXSourceCompiles)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

# Targets

# Everything except the two image_transport plugins lives in a static core
# library, so that the unit tests can link against it. The plugins themselves
# are only reachable through pluginlib and stay in the loadable module.
add_library(
    ${PROJECT_NAME}_core STATIC
    src/event_loop.cpp
    src/frame_data.cpp
    src/frame_extractor.cpp
    src/frame_injector.cpp
    src/graph_monitor.cpp
    src/host_override.cpp
    src/init.cpp
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
)
target_link_libraries(
    ${PROJECT_NAME}_core PUBLIC live555::liveMedia PkgConfig::ffmpeg
                                rclcpp::rclcpp sensor_msgs::sensor_msgs
                                std_msgs::std_msgs
)

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

add_executable(publish_rtsp_stream src/publish_rtsp_stream.cpp)
target_compile_definitions(
    publish_rtsp_stream PRIVATE "ROS_PACKAGE_NAME=\"${PROJECT_NAME}\""
)
target_link_libraries(
    publish_rtsp_stream PRIVATE rclcpp::rclcpp sensor_msgs::sensor_msgs
                                std_msgs::std_msgs
)

add_executable(
    rtsp_camera_proxy src/host_override.cpp src/rtsp_camera_proxy.cpp
)
target_compile_features(rtsp_camera_proxy PRIVATE cxx_std_20)
target_link_libraries(
    rtsp_camera_proxy PRIVATE live555::liveMedia rclcpp::rclcpp
                              sensor_msgs::sensor_msgs std_msgs::std_msgs
)

# Installation
install(
    TARGETS publish_rtsp_stream rtsp_camera_proxy ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    set(unit_tests
        test_video_codec
        test_event_loop
        test_frame_data
        test_stream_decoder
        test_stream_encoder
        test_roundtrip
        test_host_override
        test_graph_monitor
    )
    # colcon runs the test binaries in parallel, and video encoding is not cheap;
    # the generous timeout keeps a loaded build server from reporting a slow run
    # as a failure.
    foreach(test_name IN LISTS unit_tests)
        ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 180)
        if(TARGET ${test_name})
            target_link_libraries(${test_name} ${PROJECT_NAME}_core)
        endif()
    endforeach()

    # Streams video through a real loopback RTSP session, so it needs a little
    # more room than the default 60 s.
    ament_add_gtest(test_rtsp_loopback test/test_rtsp_loopback.cpp TIMEOUT 180)
    if(TARGET test_rtsp_loopback)
        target_link_libraries(test_rtsp_loopback ${PROJECT_NAME}_core)
    endif()
endif()

# Ament package handling
pluginlib_export_plugin_description_file(image_transport plugin.xml)
ament_package()
