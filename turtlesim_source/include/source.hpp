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
#include <turtlesim_source.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace zenoh
{
  namespace flow
  {

    class State
    {
    private:
      std::shared_ptr<TurtleSimSource> native_ptr;
      // zenoh::flow::GeometryMsgsTwist& operator()(const zenoh::flow::GeometryMsgsTwist& twist);
      // zenoh::flow::GeometryMsgsTwist& operator()(const zenoh::flow::GeometryMsgsTwist& twist);
    public : State();
      int Run();
      geometry_msgs::msg::Twist Data();
    };

    std::unique_ptr<State>
    initialize(const ConfigurationMap &configuration);
    zenoh::flow::GeometryMsgsTwist run(Context &context, std::unique_ptr<State> &state);

  } // namespace flow
} // namespace zenoh
