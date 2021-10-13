#!/bin/bash

set -e

source /opt/ros/foxy/setup.bash

find target/** -name "*.a" |xargs rm -rf

colcon build --merge-install
