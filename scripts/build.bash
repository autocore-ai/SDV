#!/bin/bash

set -e

source /opt/ros/foxy/setup.bash

colcon build --merge-install
