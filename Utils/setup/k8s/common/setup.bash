#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

CURDIR="$( dirname "${BASH_SOURCE[0]}" )"

source $CURDIR/kubeadm/setup.bash

source $CURDIR/minikube/setup.bash

source $CURDIR/helm/setup.bash

source $CURDIR/docker/setup.bash
