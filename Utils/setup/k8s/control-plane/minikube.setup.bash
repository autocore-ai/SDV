#!/bin/bash

set -e

if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

# export HTTP_PROXY=$http_proxy
# export HTTPS_PROXY=$https_proxy
# export NO_PROXY=$no_proxy

minikube delete -p sdv-demo

minikube start --cpus=8 --memory=12g --nodes 3 -p sdv-demo

kubectl label node sdv-demo app=host

kubectl label node sdv-demo-m02 app=autoware

kubectl label node sdv-demo-m03 app=autoware

if [ -x "$(command -v code)" ]; then
  code --install-extension ms-azuretools.vscode-docker
  code --install-extension ms-kubernetes-tools.vscode-kubernetes-tools
fi
