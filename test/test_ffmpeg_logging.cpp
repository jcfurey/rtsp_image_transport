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

/* The bridge that turns libavcodec's log callbacks into ROS log records.
 *
 * It formats into a fixed 256 byte buffer and then walks backwards over the
 * result trimming newlines, which is exactly the shape of code that overruns by
 * one on an edge case. Everything libavcodec emits goes through it, including
 * messages built from stream data, so the interesting inputs are the awkward
 * ones: longer than the buffer, exactly the buffer, empty, all newlines. */

#include "init.h"

#include <gtest/gtest.h>

#include <rcutils/logging.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/log.h>
}

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

using namespace rtsp_image_transport;

namespace
{

struct Record
{
    int severity;
    std::string name;
    std::string message;
};

std::mutex g_mutex;
std::vector<Record> g_records;
rcutils_logging_output_handler_t g_previous = nullptr;

void captureHandler(const rcutils_log_location_t*, int severity, const char* name, rcutils_time_point_value_t,
                    const char* format, va_list* args)
{
    char buffer[1024];
    va_list copy;
    va_copy(copy, *args);
    const int written = vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lock{g_mutex};
    g_records.push_back({severity, name ? name : "", written > 0 ? std::string(buffer) : std::string()});
}

class FfmpegLoggingTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        /* Installs the av_log callback under the once flag; harmless to repeat */
        global_initialize();
        g_previous = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(captureHandler);
        clear();
    }

    void TearDown() override
    {
        if (g_previous)
            rcutils_logging_set_output_handler(g_previous);
        rcutils_logging_set_logger_level("ffmpeg", RCUTILS_LOG_SEVERITY_UNSET);
        av_log_set_level(AV_LOG_ERROR);
    }

    static void clear()
    {
        std::lock_guard<std::mutex> lock{g_mutex};
        g_records.clear();
    }

    static std::vector<Record> records()
    {
        std::lock_guard<std::mutex> lock{g_mutex};
        return g_records;
    }
};

}  // namespace

TEST_F(FfmpegLoggingTest, ForwardsAnErrorToRos)
{
    av_log(nullptr, AV_LOG_ERROR, "something went wrong: %d\n", 42);

    const auto captured = records();
    ASSERT_FALSE(captured.empty()) << "the ffmpeg log callback produced no ROS record";
    EXPECT_EQ(captured.front().name, "ffmpeg");
    EXPECT_EQ(captured.front().severity, RCUTILS_LOG_SEVERITY_ERROR);
    EXPECT_NE(captured.front().message.find("something went wrong: 42"), std::string::npos)
        << captured.front().message;
}

/* The trailing newline libavcodec puts on every line would otherwise produce a
   blank line in the ROS log. */
TEST_F(FfmpegLoggingTest, StripsTrailingNewlines)
{
    av_log(nullptr, AV_LOG_ERROR, "trailing newlines\n\n\n");

    const auto captured = records();
    ASSERT_FALSE(captured.empty());
    EXPECT_EQ(captured.front().message.find('\n'), std::string::npos)
        << "a newline survived into the ROS record: " << captured.front().message;
    EXPECT_NE(captured.front().message.find("trailing newlines"), std::string::npos);
}

/* A message consisting only of newlines trims away to nothing and must be
   dropped rather than logged as an empty line — and must not walk off the front
   of the buffer while trimming. */
TEST_F(FfmpegLoggingTest, DropsAMessageThatIsNothingButNewlines)
{
    av_log(nullptr, AV_LOG_ERROR, "\n\n\n\n");
    EXPECT_TRUE(records().empty()) << "an empty line was forwarded to ROS";

    clear();
    av_log(nullptr, AV_LOG_ERROR, "%s", "");
    EXPECT_TRUE(records().empty()) << "an empty message was forwarded to ROS";
}

/* Longer than the 256 byte formatting buffer. The length vsnprintf reports is
   what the message would have been, not what fits, so using it unclamped to
   index the buffer writes past the end. */
TEST_F(FfmpegLoggingTest, TruncatesAnOverlongMessageSafely)
{
    const std::string huge(4000, 'x');
    av_log(nullptr, AV_LOG_ERROR, "%s\n", huge.c_str());

    const auto captured = records();
    ASSERT_FALSE(captured.empty());
    EXPECT_LT(captured.front().message.size(), 400u) << "the message was not truncated";
    EXPECT_GT(captured.front().message.size(), 100u) << "the message was truncated to nothing";
    EXPECT_NE(captured.front().message.find("xxxx"), std::string::npos);
}

/* Exactly the buffer size, and one either side of it: the off-by-one band. */
TEST_F(FfmpegLoggingTest, HandlesLengthsAroundTheBufferBoundary)
{
    for (std::size_t length : {254u, 255u, 256u, 257u, 258u})
    {
        clear();
        const std::string message(length, 'y');
        av_log(nullptr, AV_LOG_ERROR, "%s", message.c_str());
        const auto captured = records();
        ASSERT_FALSE(captured.empty()) << "nothing logged for length " << length;
        EXPECT_FALSE(captured.front().message.empty()) << "empty record for length " << length;
    }
}

/* Every severity libavcodec can emit has to map onto something, and the
   mapping is what decides whether a decode error is visible at all. */
TEST_F(FfmpegLoggingTest, MapsSeveritiesOntoRosLevels)
{
    /* The bridge follows the ROS logger's level: at debug it lets everything up
       to AV_LOG_INFO through, which is the only way to see the lower levels. */
    rcutils_logging_set_logger_level("ffmpeg", RCUTILS_LOG_SEVERITY_DEBUG);

    struct Case
    {
        int av_level;
        int ros_severity;
    };
    const Case cases[] = {
        {AV_LOG_FATAL, RCUTILS_LOG_SEVERITY_FATAL},
        {AV_LOG_ERROR, RCUTILS_LOG_SEVERITY_ERROR},
        {AV_LOG_WARNING, RCUTILS_LOG_SEVERITY_WARN},
        {AV_LOG_INFO, RCUTILS_LOG_SEVERITY_INFO},
    };
    for (const Case& c : cases)
    {
        clear();
        av_log(nullptr, c.av_level, "level probe\n");
        const auto captured = records();
        ASSERT_FALSE(captured.empty()) << "nothing logged for av level " << c.av_level;
        EXPECT_EQ(captured.front().severity, c.ros_severity) << "av level " << c.av_level;
    }
}

/* Below the configured level nothing should reach ROS at all, which is what
   keeps a chatty decoder from flooding the log. */
TEST_F(FfmpegLoggingTest, DropsMessagesBelowTheConfiguredLevel)
{
    av_log_set_level(AV_LOG_ERROR);
    rcutils_logging_set_logger_level("ffmpeg", RCUTILS_LOG_SEVERITY_INFO);

    av_log(nullptr, AV_LOG_DEBUG, "chatter\n");
    av_log(nullptr, AV_LOG_VERBOSE, "more chatter\n");
    EXPECT_TRUE(records().empty()) << "a filtered message still reached ROS";
}

/* libavcodec passes the AVClass of whatever object logged, and the bridge
   prints its name. A null one is the "misc" case and must not be dereferenced. */
TEST_F(FfmpegLoggingTest, LabelsMessagesWithTheAvClassName)
{
    av_log(nullptr, AV_LOG_ERROR, "no class here\n");
    auto captured = records();
    ASSERT_FALSE(captured.empty());
    EXPECT_NE(captured.front().message.find("misc"), std::string::npos) << captured.front().message;

    clear();
    /* An AVCodecContext carries a real AVClass */
    AVCodecContext* ctx = avcodec_alloc_context3(nullptr);
    ASSERT_NE(ctx, nullptr);
    av_log(ctx, AV_LOG_ERROR, "with a class\n");
    captured = records();
    avcodec_free_context(&ctx);
    ASSERT_FALSE(captured.empty());
    EXPECT_NE(captured.front().message.find("with a class"), std::string::npos);
}

TEST(FfmpegLoggingInit, GlobalInitializeIsIdempotent)
{
    /* Called from every plugin and every stream object; repeating it must not
       reinstall the callback or reopen the host override socket. */
    global_initialize();
    const int first = ros_interface_socket();
    for (int i = 0; i < 5; ++i)
        global_initialize();
    EXPECT_EQ(ros_interface_socket(), first);
}
