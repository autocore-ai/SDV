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
#include <sink.hpp>

namespace zenoh
{
  namespace flow
  {

    State::State() {}

    std::unique_ptr<State>
    initialize(const ConfigurationMap &configuration)
    {
      //
      // /!\ NOTE: `make_unique` requires "c++14"
      //

      for (auto config : configuration.map)
      {
        std::cout << "turtlesim_sink config : " << config.key.c_str() << " : " << config.value.c_str() << std::endl;
      }
      return std::make_unique<State>();
    }

    bool
    input_rule(Context &context, std::unique_ptr<State> &state, rust::Vec<Token> &tokens)
    {
      for (auto token : tokens)
      {
        if (token.status != TokenStatus::Ready)
        {
          return false;
        }
      }

      return true;
    }

    void
    run(Context &context, std::unique_ptr<State> &state, rust::Vec<Input> inputs)
    {
      for (auto input : inputs)
      {
        std::cout << "Received on <" << input.port_id.c_str() << ">: " << std::endl;
        std::cout << "\t";
        for (unsigned char c : input.data)
        {
          std::cout << unsigned(c);
        }
        std::cout << std::endl
                  << std::flush;
      }
    }

  } // namespace flow
} // namespace zenoh