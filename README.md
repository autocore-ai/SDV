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

1. Host PC:  
  - CPU: x86_64 3 Core or above  
  - RAM: 8G+  
  - OS: Ubuntu 18.04+  
  - cable Ethernet

2. Autoware PC:  
  - CPU: x86_64 8 Core or above  
  - RAM: 16G+  
  - Disk: 30G+ free space  
  - OS: Ubuntu 18.04+  
  - cable Ethernet

3. PCU clients x N  

### Environment setup

1. Host PC：  

  - `$ source Utils/setup/k8s/control-plane/setup.bash`  
  Concole will output the information like "kubeadm join xxx", please use this information for clients to join in later steps.

2. Autoware PC:

  - `$ source Utils/setup/k8s/worker/setup.bash`  
  - `$ kubeadm join XXX`  
    Please find the "xxx" in the concole output of Host PC as described in step 1.

3. PCU:  

  - `$ source Utils/setup/k8s/worker/setup.bash`
  - `$ kubeadm join XXX`  
    Please find the "xxx" in the concole output of Host PC as described in step 1.

### Deploy workloads with config file

```bash
kubectl apply -f https://raw.githubusercontent.com/autocore-ai/SDV/preview/sdv_demo.yaml
```

### Use CloudViewer to display scene and send commands.

Just open [CloudViewer](https://autocore-ai.github.io/CloudViewer/)
