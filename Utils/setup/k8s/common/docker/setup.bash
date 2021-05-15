#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

if [ -z "$(command -v curl)" ]; then
    sudo apt install -y curl
fi

if [ -z "$(command -v docker)" ]; then
  if [ $(arch) == 'x86_64' ]; then
    sudo curl https://get.docker.com | sudo sh
  elif [ $(arch) == 'aarch64' ]; then
    echo "ARM64 arch needs install docker manually"
  else
    echo "Unknow arch"
    exit 1
  fi
fi

sudo usermod -aG docker $USER

echo '{"exec-opts": ["native.cgroupdriver=systemd"]}' | sudo tee /etc/docker/daemon.json

sudo systemctl daemon-reload

sudo service docker restart
