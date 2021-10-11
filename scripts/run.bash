#!/bin/bash

set -e

source ./install/setup.bash

ros2 run turtlesim turtlesim_node & ros2 run turtlesim turtle_teleop_key
