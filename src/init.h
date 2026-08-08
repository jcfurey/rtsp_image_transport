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
#ifndef RTSP_IMAGE_TRANSPORT_INIT_H_
#define RTSP_IMAGE_TRANSPORT_INIT_H_

#define FKIE_VERSION_TUPLE(major, minor, patch) (65536 * (major) + 256 * (minor) + patch)

/* image_transport 6.4 added plugin entry points taking node interfaces rather
   than a node, and stopped calling the old ones. 7.0 removed the old ones
   altogether, so overriding them there does not compile. The two boundaries are
   distinct: 6.4 decides which entry point does the work, 7.0 decides whether the
   legacy one may be declared at all. */
#define RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES \
    (CURRENT_IMAGE_TRANSPORT_VERSION >= FKIE_VERSION_TUPLE(6, 4, 0))
#define RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API \
    (CURRENT_IMAGE_TRANSPORT_VERSION < FKIE_VERSION_TUPLE(7, 0, 0))

namespace rtsp_image_transport
{

void global_initialize();
int ros_interface_socket();

}  // namespace rtsp_image_transport

#endif
