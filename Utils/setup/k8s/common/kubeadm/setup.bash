#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

if [ -x "$(command -v kubeadm)" ] && [ -x "$(command -v kubectl)" ]; then
    return
fi

if [ -z "$(command -v curl)" ]; then
    sudo apt install -y curl
fi

sudo curl -fsSLo /usr/share/keyrings/kubernetes-archive-keyring.gpg https://packages.cloud.google.com/apt/doc/apt-key.gpg

echo "deb [signed-by=/usr/share/keyrings/kubernetes-archive-keyring.gpg] https://apt.kubernetes.io/ kubernetes-xenial main" | sudo tee /etc/apt/sources.list.d/kubernetes.list

sudo apt update

sudo apt install -y kubelet kubeadm kubectl

sudo swapoff -a

sudo sed -i '/ swap / s/^/#/' /etc/fstab
