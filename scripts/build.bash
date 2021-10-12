#!/bin/bash

set -e

source /opt/ros/foxy/setup.bash

rm target/**/*.a

colcon build --merge-install

source ./scripts/run.bash
