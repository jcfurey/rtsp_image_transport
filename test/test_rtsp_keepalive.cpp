// SPDX-License-Identifier: Apache-2.0
#include "stream_client.h"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

using namespace rtsp_image_transport;
using namespace std::chrono_literals;

namespace
{
// A camera with a two-second control session and no media. Keeping media and
// RTCP out of this test proves that RTSP liveness is supervised independently.
class ControlServer
{
  public:
    explicit ControlServer(bool reject_get = false, bool ignore_keepalive = false)
        : reject_get_(reject_get), ignore_keepalive_(ignore_keepalive)
    {
        listener_ = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        socklen_t length = sizeof(address);
        if (listener_ < 0 || bind(listener_, reinterpret_cast<sockaddr *>(&address), length) != 0 ||
            listen(listener_, 1) != 0 ||
            getsockname(listener_, reinterpret_cast<sockaddr *>(&address), &length) != 0)
        {
            if (listener_ >= 0)
                close(listener_);
            throw std::runtime_error("cannot listen for the RTSP control test");
        }
        url_ = "rtsp://127.0.0.1:" + std::to_string(ntohs(address.sin_port)) + "/camera/";
        thread_ = std::thread([this] { run(); });
    }

    ~ControlServer()
    {
        stop_ = true;
        thread_.join();
        if (peer_ >= 0)
            close(peer_);
        close(listener_);
    }

    const std::string &url() const
    {
        return url_;
    }
    std::atomic<unsigned> get_parameters{0}, options{0};

  private:
    bool sendAll(const std::string &data)
    {
        std::size_t sent = 0;
        while (sent < data.size())
        {
            const auto n = send(peer_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
            if (n <= 0)
                return false;
            sent += static_cast<std::size_t>(n);
        }
        return true;
    }

    bool respond(const std::string &request)
    {
        const auto method = request.substr(0, request.find(' '));
        const auto seq_start = request.find("CSeq: ");
        if (seq_start == std::string::npos)
            return false;
        const auto seq = request.substr(seq_start + 6, request.find("\r\n", seq_start) - seq_start - 6);
        std::string headers = "Session: 12345678;timeout=2\r\n";
        std::string body;
        if (method == "DESCRIBE")
        {
            headers += "Content-Type: application/sdp\r\nContent-Base: " + url_ + "\r\n";
            body = "v=0\r\no=- 1 1 IN IP4 127.0.0.1\r\ns=control test\r\nt=0 0\r\n"
                   "m=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\na=control:track1\r\n";
        }
        else if (method == "SETUP")
            headers += "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n";
        else if (method == "GET_PARAMETER")
        {
            ++get_parameters;
            if (ignore_keepalive_)
                return true;
            if (reject_get_)
                return sendAll("RTSP/1.0 405 Method Not Allowed\r\nCSeq: " + seq + "\r\n\r\n");
        }
        else if (method == "OPTIONS")
        {
            ++options;
            headers += "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n";
        }
        return sendAll("RTSP/1.0 200 OK\r\nCSeq: " + seq + "\r\n" + headers +
                       "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body);
    }

    void run()
    {
        std::string pending;
        while (!stop_)
        {
            pollfd ready{peer_ >= 0 ? peer_ : listener_, POLLIN, 0};
            if (poll(&ready, 1, 50) <= 0)
                continue;
            if (peer_ < 0)
            {
                peer_ = accept(listener_, nullptr, nullptr);
                continue;
            }
            char buffer[4096];
            const auto n = recv(peer_, buffer, sizeof(buffer), 0);
            if (n <= 0)
                return;
            pending.append(buffer, static_cast<std::size_t>(n));
            if (pending.size() > 65536)
                return;
            for (auto end = pending.find("\r\n\r\n"); end != std::string::npos;
                 end = pending.find("\r\n\r\n"))
            {
                const auto request = pending.substr(0, end + 4);
                pending.erase(0, end + 4);
                if (!respond(request))
                    return;
            }
        }
    }

    const bool reject_get_, ignore_keepalive_;
    std::atomic<bool> stop_{false};
    int listener_ = -1, peer_ = -1;
    std::string url_;
    std::thread thread_;
};

template <class Predicate> bool waitFor(Predicate predicate)
{
    const auto deadline = std::chrono::steady_clock::now() + 4s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(10ms);
    return predicate();
}
} // namespace

TEST(RtspKeepalive, RefreshesTheAdvertisedSessionWithoutMedia)
{
    ControlServer server;
    std::atomic<bool> started{false}, failed{false};
    auto client = StreamClient::create("keepalive", server.url());
    client->setSessionStartedHandler([&] { started = true; });
    client->setSessionFailedHandler([&](int, const std::string &) { failed = true; });
    client->setRtpOverTcp(true);
    client->connect();
    ASSERT_TRUE(waitFor([&] { return started.load(); }));
    EXPECT_TRUE(waitFor([&] { return server.get_parameters >= 2; }));
    client->disconnect();
    EXPECT_FALSE(failed);
}

TEST(RtspKeepalive, CamerasRejectingGetParameterFallBackToOptions)
{
    ControlServer server(true);
    std::atomic<bool> failed{false};
    auto client = StreamClient::create("keepalive_options", server.url());
    client->setSessionFailedHandler([&](int, const std::string &) { failed = true; });
    client->setRtpOverTcp(true);
    client->connect();
    EXPECT_TRUE(waitFor([&] { return server.options >= 2; }));
    client->disconnect();
    EXPECT_EQ(server.get_parameters, 1u);
    EXPECT_FALSE(failed);
}

TEST(RtspKeepalive, MissingControlResponsesTimeoutEvenWithMediaWatchdogDisabled)
{
    ControlServer server(false, true);
    std::atomic<int> failure{0};
    auto client = StreamClient::create("keepalive_timeout", server.url());
    client->setSessionFailedHandler([&](int code, const std::string &) { failure = code; });
    client->setRtpOverTcp(true);
    client->connect();
    EXPECT_TRUE(waitFor([&] { return failure.load() != 0; }));
    client->disconnect();
    EXPECT_EQ(failure, 408);
}
