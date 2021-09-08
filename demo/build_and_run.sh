#!/bin/bash

set -e

cargo build \
--package demo-source \
--package demo-sink

source ./demo/run_demo.sh
