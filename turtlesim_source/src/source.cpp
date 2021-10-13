//
// Copyright (c) 2017, 2021 ADLINK Technology Inc.
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ADLINK zenoh team, <zenoh@adlink-labs.tech>
//

#include <iostream>
#include <source.hpp>

namespace zenoh
{
  namespace flow
  {

    using byte_t = unsigned char;

    State::State()
    {
      native_ptr = std::make_shared<TurtleSimSource>(0, nullptr);
    }

    int State::Run()
    {
      auto ret = native_ptr->Run();
      if (ret <= 0)
      {
        native_ptr->Quit();
      }
      return ret;
    }

    geometry_msgs::msg::Twist State::Data()
    {
      return native_ptr->twist;
    }

    std::unique_ptr<State>
    initialize(const ConfigurationMap &configuration)
    {
      return std::make_unique<State>();
    }

    rust::Vec<Output>
    run(Context &context, std::unique_ptr<State> &state)
    {
      state->Run();

      state->Data();

      rust::Vec<byte_t> tick = {1};

      Output output{"tick", tick};

      rust::Vec<Output> results{output};
      return results;
    }
  } // namespace flow
} // namespace zenoh
