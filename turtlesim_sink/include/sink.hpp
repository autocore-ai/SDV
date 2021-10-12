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

#pragma once
#include <wrapper.hpp>

namespace zenoh {
namespace flow {

class State {
public:
  State();
};

std::unique_ptr<State> initialize(const ConfigurationMap &configuration);

bool input_rule(Context                &context,
                std::unique_ptr<State> &state,
                rust::Vec<Token>       &tokens);

void run(Context                &context,
         std::unique_ptr<State> &state,
         rust::Vec<Input>        inputs);

} // namespace flow
} // namespace zenoh
