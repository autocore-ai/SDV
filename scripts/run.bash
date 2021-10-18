#!/bin/bash

set -e

source ./install/setup.bash

# export RUST_LOG=debug

runtime --graph-file turtlesim.yaml --runtime local

# ros2 run zf_turtlesim turtlesim_source_native_exe
