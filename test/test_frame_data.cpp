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
#include "frame_data.h"

#include <gtest/gtest.h>

#include <vector>

extern "C"
{
#include <libavcodec/avcodec.h>
}

using namespace rtsp_image_transport;

TEST(FrameData, CopiesPayloadAndKeepsStamp)
{
    std::vector<unsigned char> payload{0x00, 0x00, 0x00, 0x01, 0x65, 0xde, 0xad, 0xbe, 0xef};
    rclcpp::Time stamp(1700000000123456789LL);
    FrameData frame(payload.data(), payload.size(), stamp);

    ASSERT_EQ(frame.length(), payload.size());
    EXPECT_EQ(frame.stamp().nanoseconds(), stamp.nanoseconds());
    EXPECT_EQ(std::vector<unsigned char>(frame.data(), frame.data() + frame.length()), payload);

    /* The frame owns its bytes: mutating the source afterwards must not show */
    payload[4] = 0x41;
    EXPECT_EQ(frame.data()[4], 0x65);
}

TEST(FrameData, AppendsZeroedDecoderPadding)
{
    /* FFmpeg reads up to AV_INPUT_BUFFER_PADDING_SIZE bytes past the end of a
       packet; those bytes must exist and be zero or the decoder can misparse
       the tail of the bitstream. */
    std::vector<unsigned char> payload(64, 0xAB);
    FrameData frame(payload.data(), payload.size(), rclcpp::Time(0));
    for (int i = 0; i < AV_INPUT_BUFFER_PADDING_SIZE; ++i)
        EXPECT_EQ(frame.data()[frame.length() + i], 0u) << "padding byte " << i << " is not zero";
}

TEST(FrameData, HandlesEmptyPayload)
{
    FrameData frame(nullptr, 0, rclcpp::Time(42));
    EXPECT_EQ(frame.length(), 0u);
    ASSERT_NE(frame.data(), nullptr);
    for (int i = 0; i < AV_INPUT_BUFFER_PADDING_SIZE; ++i)
        EXPECT_EQ(frame.data()[i], 0u);
}

TEST(FrameData, StampCanBeOverridden)
{
    std::vector<unsigned char> payload(8, 0x11);
    FrameData frame(payload.data(), payload.size(), rclcpp::Time(1));
    frame.setStamp(rclcpp::Time(999));
    EXPECT_EQ(frame.stamp().nanoseconds(), 999);
}
