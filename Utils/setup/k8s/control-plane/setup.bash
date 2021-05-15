#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

sudo apt update

CURDIR="$( dirname "${BASH_SOURCE[0]}" )"

source $CURDIR/../common/setup.bash

sudo kubeadm init

mkdir -p $HOME/.kube

sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config

sudo chown $(id -u):$(id -g) $HOME/.kube/config
