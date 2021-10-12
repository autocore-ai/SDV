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
      native_ptr = std::make_shared<TurtleSimSource>();
    }

    // std::unique_ptr<State>
    // initialize(const ConfigurationMap &configuration)
    // {
    //   return std::make_unique<State>();
    // }

    // rust::Vec<Output>
    // run(Context &context, std::unique_ptr<State> &state)
    // {
    //   std::string input;

    //   std::cout << "Press ENTER.";
    //   std::getline(std::cin, input);
    //   std::cout << std::endl;

    //   rust::Vec<byte_t> tick = {1};

    //   Output output{"tick", tick};

    //   rust::Vec<Output> results{output};
    //   return results;
    // }
  } // namespace flow
} // namespace zenoh
