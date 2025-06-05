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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
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
add_library(
    ${PROJECT_NAME} MODULE
    src/frame_data.cpp
    src/frame_extractor.cpp
    src/frame_injector.cpp
    src/graph_monitor.cpp
    src/host_override.cpp
    src/init.cpp
    src/publisher_plugin.cpp
    src/stream_client.cpp
    src/stream_decoder.cpp
    src/stream_encoder.cpp
    src/stream_server.cpp
    src/subscriber_plugin.cpp
    src/video_codec.cpp
)
generate_export_header(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE $<$<BOOL:${LIVE555_HAS_H265_SUPPORT}>:LIVE555_HAS_H265_SUPPORT>
            $<$<BOOL:${LIVE555_HAS_VPX_SUPPORT}>:LIVE555_HAS_VPX_SUPPORT>
            $<$<BOOL:${LIVE555_HAS_AV1_SUPPORT}>:LIVE555_HAS_AV1_SUPPORT>
            $<$<BOOL:${FFMPEG_HAS_HWFRAME_SUPPORT}>:FFMPEG_HAS_HWFRAME_SUPPORT>
            CURRENT_RCLCPP_VERSION=${computed_rclcpp_version}
)
set_target_properties(
    ${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden"
                               VISIBILITY_INLINES_HIDDEN ON
)
target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE cv_bridge::cv_bridge image_transport::image_transport
            live555::liveMedia PkgConfig::ffmpeg pluginlib::pluginlib
            rclcpp::rclcpp
)

# Make the "RTSP Only" test image embeddable
add_custom_command(
    OUTPUT rtsp_only.o
    MAIN_DEPENDENCY rtsp_only.png
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND ld -z noexecstack -r -b binary -o
            ${CMAKE_CURRENT_BINARY_DIR}/rtsp_only.o rtsp_only.png
)

add_executable(publish_rtsp_stream rtsp_only.o src/publish_rtsp_stream.cpp)
target_compile_definitions(
    publish_rtsp_stream PRIVATE "ROS_PACKAGE_NAME=\"${PROJECT_NAME}\""
)
target_link_libraries(
    publish_rtsp_stream PRIVATE cv_bridge::cv_bridge rclcpp::rclcpp
)

add_executable(
    rtsp_camera_proxy rtsp_only.o src/host_override.cpp
                      src/rtsp_camera_proxy.cpp
)
target_compile_features(rtsp_camera_proxy PRIVATE cxx_std_20)
target_link_libraries(
    rtsp_camera_proxy PRIVATE cv_bridge::cv_bridge live555::liveMedia
                              rclcpp::rclcpp
)

# Installation
install(
    TARGETS publish_rtsp_stream rtsp_camera_proxy ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

# Ament package handling
pluginlib_export_plugin_description_file(image_transport plugin.xml)
ament_package()
