// SPDX-License-Identifier: Apache-2.0
#ifndef RTSP_IMAGE_TRANSPORT_CALLBACK_GATE_H_
#define RTSP_IMAGE_TRANSPORT_CALLBACK_GATE_H_

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

namespace rtsp_image_transport
{

// Removing a ROS timer or parameter callback does not revoke a callback the
// executor already took. Captured by value, this gate outlives the plugin and
// lets that callback return without dereferencing its former owner.
struct CallbackGate : std::enable_shared_from_this<CallbackGate>
{
    std::recursive_mutex mutex;
    bool active = true;

    void close()
    {
        std::lock_guard<std::recursive_mutex> lock{mutex};
        active = false;
    }

    template <class Function> auto wrap(Function function)
    {
        return [gate = shared_from_this(), function = std::move(function)](auto&&... args) mutable
        {
            std::lock_guard<std::recursive_mutex> lock{gate->mutex};
            if (gate->active)
                std::invoke(function, std::forward<decltype(args)>(args)...);
        };
    }
};

} // namespace rtsp_image_transport

#endif
