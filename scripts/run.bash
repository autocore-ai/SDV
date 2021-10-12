#!/bin/bash

set -e

source ./install/setup.bash

runtime --graph-file turtlesim.yaml --runtime turlesim
