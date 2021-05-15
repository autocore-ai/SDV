#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

if [ -x "$(command -v minikube)" ]; then
    return
fi

if [ $(arch) == 'x86_64' ]; then
  DEB_URL="https://storage.googleapis.com/minikube/releases/latest/minikube_latest_amd64.deb"
elif [ $(arch) == 'aarch64' ]; then
  DEB_URL="https://storage.googleapis.com/minikube/releases/latest/minikube_latest_arm64.deb"
else
  echo "Unknow arch"
  exit 1
fi

TEMP_DEB="$(mktemp)"

wget -O "$TEMP_DEB" "$DEB_URL"

sudo dpkg -i "$TEMP_DEB"

rm -f "$TEMP_DEB"
