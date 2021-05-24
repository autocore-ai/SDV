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

- Host:

  - Host PC:
    - CPU: x86_64 4 Core or above
    - RAM: 8G+
    - Disk: 30G+ free space
    - OS: Ubuntu 18.04+
    - cable Ethernet

- For each Worker (default sets 2 workers in cluster):

  - [AutoCore PCU][autocore pcu]

    - cable Ethernet

      **OR**

  - Worker PC (**Needs when you don't have [AutoCore PCU][autocore pcu]**):
    - CPU: x86_64 8 Core or above
    - RAM: 8G+
    - Disk: 30G+ free space
    - OS: Ubuntu 18.04+
    - cable Ethernet

### Environment setup

1. Host PC:

- ```bash
  $ source Utils/setup/k8s/control-plane/setup.bash
  ```
  Concole will output the information like `kubeadm join xxx`, please use this information for clients to join in later steps.

1. [AutoCore PCU][autocore pcu] **OR** Worker PC:

- ```bash
  $ source Utils/setup/k8s/worker/setup.bash
  ```
- ```bash
  $ sudo kubeadm join xxx
  ```
  Please find the `xxx` in the concole output of Host PC as described in step 1.

1. Back to Host PC

- For each worker node, label them:
  ```bash
  $ kubectl label node <worker_node_name> app=autoware
  ```

### Deploy workloads with config file

Default configs 2 workers in cluster, and if you have two works hardware, just run the default command:

```bash
$ kubectl apply -f https://raw.githubusercontent.com/autocore-ai/SDV/develop/sdv_demo.yaml
```

And if you have other custom deployment, Just download sdv_demo.yaml and fix the `replicas: 2` to your worker count N:

```diff
spec:
- replicas: 2
+ replicas: N
  selector:
```

then use this config file:

```bash
$ kubectl apply -f sdv_demo.yaml
```

### Use [CloudViewer][cloudviewer] to display scene and send commands.

Open [CloudViewer][cloudviewer] with [Chromium][chromium] based browser

- Config IP Address to Host PC IP (defaults `127.0.0.1`)

- Move viewer with key `WSAD` and mouse midde and right key.

- Click on vehicle and traffic light to inspect and send commands.

[autocore pcu]: https://github.com/autocore-ai/autocore_pcu_doc
[sdv_demo.yaml]: https://raw.githubusercontent.com/autocore-ai/SDV/develop/sdv_demo.yaml
[cloudviewer]: https://autocore-ai.github.io/CloudViewer/
[chromium]: https://www.chromium.org/
