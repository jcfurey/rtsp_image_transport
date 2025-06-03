/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
#include "init.h"

#include "host_override.h"

#include <rclcpp/logging.hpp>

#include <mutex>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/log.h>
}

namespace rtsp_image_transport
{

namespace
{

void ffmpeg_log_to_ros(void* avcl, int level, const char* fmt, va_list ap)
{
    static rclcpp::Logger logger = rclcpp::get_logger("ffmpeg");
    int effective_log_level =
        logger.get_effective_level() == rclcpp::Logger::Level::Debug ? AV_LOG_INFO : av_log_get_level();
    if (level > effective_log_level)
        return;
    char buf[256];
    const char* class_name = "misc";
    if (avcl)
        class_name = (*static_cast<AVClass**>(avcl))->class_name;
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (len <= 0)
        return;
    if (len >= static_cast<int>(sizeof(buf)))
        len = sizeof(buf) - 1;
    while (len > 0 && (buf[len - 1] == '\0' || buf[len - 1] == '\n'))
        len--;
    if (len == 0)
        return;
    buf[len] = '\0';
    switch (level)
    {
        case AV_LOG_PANIC:
        case AV_LOG_FATAL:
            RCLCPP_FATAL(logger, "[%s] %s", class_name, buf);
            break;
        case AV_LOG_ERROR:
            RCLCPP_ERROR(logger, "[%s] %s", class_name, buf);
            break;
        case AV_LOG_WARNING:
            RCLCPP_WARN(logger, "[%s] %s", class_name, buf);
            break;
        case AV_LOG_INFO:
            RCLCPP_INFO(logger, "[%s] %s", class_name, buf);
            break;
        default:
            RCLCPP_DEBUG(logger, "[%s] %s", class_name, buf);
            break;
    }
}

int ros_interface_socket_ = -1;

void do_global_initialize()
{
#if LIBAVCODEC_VERSION_MAJOR < 58
    /* These functions are deprecated since FFmpeg 4.0 */
    av_register_all();
    avcodec_register_all();
#endif
    av_log_set_callback(ffmpeg_log_to_ros);
    av_log_set_level(AV_LOG_ERROR);
    if (const char* node = getenv("ROS_HOSTNAME"))
    {
        ros_interface_socket_ = create_host_override_socket(node);
    }
    else if (const char* node = getenv("ROS_IP"))
    {
        ros_interface_socket_ = create_host_override_socket(node, true);
    }
    if (ros_interface_socket_ >= 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rtsp_image_transport"),
                    "ROS_HOSTNAME/ROS_IP override: RTSP server will advertise IP "
                    "address %s",
                    socket_bound_address(ros_interface_socket_).c_str());
    }
}

}  // namespace

std::once_flag global_init;

void global_initialize()
{
    std::call_once(global_init, do_global_initialize);
}

int ros_interface_socket()
{
    return ros_interface_socket_;
}

}  // namespace rtsp_image_transport
