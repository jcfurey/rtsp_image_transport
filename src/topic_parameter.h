/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021-2025 Fraunhofer FKIE
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
#ifndef RTSP_IMAGE_TRANSPORT_TOPIC_PARAMETER_H_
#define RTSP_IMAGE_TRANSPORT_TOPIC_PARAMETER_H_

#include <rclcpp/node.hpp>

#include <algorithm>
#include <string>

namespace rtsp_image_transport
{

inline std::string topicParameterBase(const std::string& node_namespace, const std::string& base_topic,
                                      const std::string& transport)
{
    std::string parameter = base_topic;

    /* image_transport may pass either a resolved absolute topic or the original
       relative topic. Only remove the node namespace when it is actually a
       prefix; blindly removing namespace.length() corrupts relative names. */
    if (node_namespace != "/" && parameter.starts_with(node_namespace)
        && (parameter.size() == node_namespace.size() || parameter[node_namespace.size()] == '/'))
    {
        parameter.erase(0, node_namespace.size());
    }
    while (!parameter.empty() && parameter.front() == '/')
        parameter.erase(parameter.begin());

    std::replace(parameter.begin(), parameter.end(), '/', '.');
    if (!parameter.empty())
        parameter.push_back('.');
    parameter += transport;
    return parameter;
}

inline std::string topicParameterBase(const rclcpp::Node& node, const std::string& base_topic,
                                      const std::string& transport)
{
    return topicParameterBase(node.get_effective_namespace(), base_topic, transport);
}

}  // namespace rtsp_image_transport

#endif
