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
#include "topic_parameter.h"

#include <gtest/gtest.h>

using namespace rtsp_image_transport;

/* The parameter name a topic maps to. image_transport hands over either the
   resolved absolute topic or the original relative one, depending on version
   and entry point, so both have to land on the same parameter. */
TEST(TopicParameter, RelativeAndAbsoluteTopicsAgree)
{
    EXPECT_EQ(topicParameterBase("/", "camera/image", "rtsp"), "camera.image.rtsp");
    EXPECT_EQ(topicParameterBase("/", "/camera/image", "rtsp"), "camera.image.rtsp");
    EXPECT_EQ(topicParameterBase("/robot", "/robot/camera/image", "rtsp"), "camera.image.rtsp");
    EXPECT_EQ(topicParameterBase("/robot", "camera/image", "rtsp"), "camera.image.rtsp");
}

/* A namespace that is a string prefix of the topic but not a path prefix must
   not be stripped: removing namespace.length() characters blindly corrupts the
   name, which is what the starts_with plus separator check prevents. */
TEST(TopicParameter, NamespaceIsOnlyStrippedOnAPathBoundary)
{
    EXPECT_EQ(topicParameterBase("/cam", "/camera/image", "rtsp"), "camera.image.rtsp");
    EXPECT_EQ(topicParameterBase("/cam", "/cam/image", "rtsp"), "image.rtsp");
    /* The namespace itself, with nothing below it */
    EXPECT_EQ(topicParameterBase("/cam", "/cam", "rtsp"), "rtsp");
}

TEST(TopicParameter, HandlesEmptyAndRootTopics)
{
    EXPECT_EQ(topicParameterBase("/", "", "rtsp"), "rtsp");
    EXPECT_EQ(topicParameterBase("/", "/", "rtsp"), "rtsp");
    EXPECT_EQ(topicParameterBase("/", "image", "rtsp"), "image.rtsp");
}

/* resolve_topic_name() always returns an absolute name, so a check against the
   relative default can never match. Measured on Jazzy: "image" resolves to
   "/image" at the root and "/robot/image" under a namespace. */
TEST(TopicParameter, DetectsAnUnremappedTopic)
{
    EXPECT_EQ(unremappedTopicName("/", "image"), "/image");
    EXPECT_EQ(unremappedTopicName("/robot", "image"), "/robot/image");
    EXPECT_EQ(unremappedTopicName("/robot/", "image"), "/robot/image");

    EXPECT_FALSE(topicWasRemapped("/image", "/", "image"));
    EXPECT_FALSE(topicWasRemapped("/robot/image", "/robot", "image"));

    EXPECT_TRUE(topicWasRemapped("/cam/front", "/", "image"));
    EXPECT_TRUE(topicWasRemapped("/robot/cam/front", "/robot", "image"));
    /* The relative name itself is never what resolution produces, so if it ever
       shows up something else is wrong — but it is certainly not a remap. */
    EXPECT_TRUE(topicWasRemapped("image", "/", "image"));
}
