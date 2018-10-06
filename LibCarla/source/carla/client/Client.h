// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/detail/Client.h"

namespace carla {
namespace client {

  class Client {
  public:

    /// Construct a carla client.
    ///
    /// @param host IP address of the host machine running the simulator.
    /// @param port TCP port to connect with the simulator.
    /// @param worker_threads number of asynchronous threads to use, or 0 to use
    ///        all available hardware concurrency.
    explicit Client(
        const std::string &host,
        uint16_t port,
        size_t worker_threads = 0u);

    void SetTimeout(time_duration timeout) {
      _client_state->SetTimeout(timeout);
    }

    std::string GetClientVersion() const {
      return _client_state->GetClientVersion();
    }

    std::string GetServerVersion() const {
      return _client_state->GetServerVersion();
    }

    bool Ping() const {
      return _client_state->Ping();
    }

    World GetWorld() const {
      return _client_state->GetWorld();
    }

  private:

    SharedPtr<detail::Client> _client_state;
  };

} // namespace client
} // namespace carla
