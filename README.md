# AutoCore SDV Project

![Logo](https://user-images.githubusercontent.com/71419791/117961292-88465b80-b350-11eb-9cb5-221226b419c9.png "AutoCore")

For business and partnership, please visit our website: [www.autocore.ai](http://www.autocore.ai "AutoCore Homepage").

## Table of Contents

1. [Overview](#overview)  
2. [Quick Start Guide](#quick-start-guide)


## Overview

SDV (Software Defined Vehicle) project is a cooperative project between Autocore and Futurewei, which aims at providing the technology-consulting services by means of this 100 percent open source software stack with reference design for SDVs. Cloud-Edge service is also integrated as a part of this project to extend the V2X capabilities.

The SDV software stack is based on open source Autoware/ROS2/DDS and Zenoh, where DDS/Zenoh act as the E2E Vehicle-Edge-Cloud middleware layer for the SDV platform. Thru the integration with Futurewei’ s open source KubeEdge project, the SDV platform will leverage Cloud Native Ecosystem tools to provide management, monitoring and software LCM (Life-Cycle-Management) functions.

The system architecture of SDV platform project is shown in the figure below:

![Architecture](https://user-images.githubusercontent.com/7805397/112237928-a2cc5480-8c7e-11eb-8917-7a23a9f9acfb.png "Architecture")

The software modules in host container runtime are as follows:

![](https://user-images.githubusercontent.com/7805397/112241214-c98d8980-8c84-11eb-8115-91281f22ac07.png)

And the PCU Container runtime acts as the Domain controller in vehicle with the following software modules:

![](https://user-images.githubusercontent.com/7805397/112241219-cd211080-8c84-11eb-8cd3-e7db20d08565.png)

## Quick Start Guide

### Hardware requirement

- CPU: x86_64 8 Core or above  
- RAM: 16G+  
- Disk: 30G+ free space  
- OS: Ubuntu 18.04+

### Install [`docker`](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

**Then log out and log in with current user**

### Install [`kubectl`](https://kubernetes.io/docs/tasks/tools/install-kubectl-linux/#install-using-native-package-management)

```bash
sudo curl -fsSLo /usr/share/keyrings/kubernetes-archive-keyring.gpg https://packages.cloud.google.com/apt/doc/apt-key.gpg
echo "deb [signed-by=/usr/share/keyrings/kubernetes-archive-keyring.gpg] https://apt.kubernetes.io/ kubernetes-xenial main" | sudo tee /etc/apt/sources.list.d/kubernetes.list
sudo apt-get update
sudo apt-get install -y kubectl
```

### Install [`minikube`](https://minikube.sigs.k8s.io/docs/start/#what-youll-need)

```bash
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube_latest_amd64.deb
sudo dpkg -i minikube_latest_amd64.deb
```

### Create cluster

```bash
minikube start --cpus=8 --memory=16g
```

### Deploy workloads with config file

```bash
kubectl apply -f https://raw.githubusercontent.com/autocore-ai/SDV/preview/sdv_demo.yaml
```

### Use CloudViewer to display scene and send commands.

Just open [CloudViewer](https://autocore-ai.github.io/CloudViewer/)
