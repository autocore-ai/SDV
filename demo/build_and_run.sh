#!/bin/bash

set -e

cargo build

source ./demo/run_demo.sh
