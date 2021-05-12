# SDV

## 快速开始 Demo

### 环境准备

- 8 Core && 16G+ RAM && 30G Disk x86_64 pc
- Ubuntu 18.04+ OS

### 安装 [`minikube`](https://minikube.sigs.k8s.io/docs/start/#what-youll-need)

```bash
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube_latest_amd64.deb
sudo dpkg -i minikube_latest_amd64.deb
```

### 创建集群

```bash
minikube start --cpus=8 --memory=16g
```

### 启动集群

```bash
kubectl apply -f https://raw.githubusercontent.com/autocore-ai/SDV/preview/sdv_demo.yaml
```

### CloudViewer 交互

使用 Chromium 内核浏览器打开[CloudViewer](https://autocore-ai.github.io/CloudViewer/)
