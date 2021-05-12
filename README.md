# AutoCore SDV Project

![autoCore](https://autocore.ai/img/logo.webp "AutoCore")

For business and partnership, please visit our website: [www.autocore.ai](http://www.autocore.ai "AutoCore Homepage").

## Table of Contents

1. [Overview](#overview)  
2. [Quick Start Guide](#quick-start-guide)


## Overview

SDV project is a cooperative project with Futurewei, which will provide the technology-consulting services to develop and promote the 100 percent open source SDV (Software Defined Vehicle) software stack with reference design, with native integration of Cloud-Edge service capability. The SDV software stack is based on open source Autoware/ROS2/DDS and Zenoh, where DDS/Zenoh form an E2E Vehicle-Edge-Cloud middleware layer for the SDV platform. Thru the integration with Futurewei’ s open source KubeEdge project, the SDV platform will leverage Cloud Native Ecosystem tools to provide management, monitoring and software LCM (Life-Cycle-Management) functions.

The overall SDV platform solution architecture is shown below:

![Architecture](https://user-images.githubusercontent.com/7805397/112237928-a2cc5480-8c7e-11eb-8917-7a23a9f9acfb.png "Architecture")

![](https://user-images.githubusercontent.com/7805397/112241214-c98d8980-8c84-11eb-8115-91281f22ac07.png)

![](https://user-images.githubusercontent.com/7805397/112241219-cd211080-8c84-11eb-8cd3-e7db20d08565.png)

## Quick Start Guide

### Hardware requirement

- 8 Core && 16G+ RAM && 30G Disk x86_64 pc
- Ubuntu 18.04+ OS

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
