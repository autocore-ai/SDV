#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

if [ -x "$(command -v helm)" ]; then
    return
fi

curl https://baltocdn.com/helm/signing.asc | sudo apt-key add -

sudo apt install -y apt-transport-https

echo "deb https://baltocdn.com/helm/stable/debian/ all main" | sudo tee /etc/apt/sources.list.d/helm-stable-debian.list

sudo apt update

sudo apt install -y helm
