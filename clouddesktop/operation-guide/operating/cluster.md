# Cluster Management

We administrate all our operations through the use of `kubectl`.

## Prerequisites

Read up on the following materials,

- [`K8s` Concepts](https://kubernetes.io/docs/concepts/)
- [`kubectl` cheatsheet](https://kubernetes.io/docs/reference/kubectl/cheatsheet/)

## Requirements

- `kubectl` [Installation Guide](https://kubernetes.io/docs/tasks/tools/)

## Setup

To access the K8s cluster, you will need to have a `k3s.yaml` credential file. It can be obtained by ssh into the `master` node of the cluster, under the directory `/etc/rancher/k3s/k3s.yaml`.

Once you have obtained the `k3s.yaml` file, make the following modification,

```diff
# Update with the IP of the master node
- server: https://localhost:6443
+ server: https://123.123.123.123:6443
```

After the modification, this file is ready for use. Update your shell to always use this file,

```bash
export KUBECONFIG=/.../k3s.yaml
```

To confirm it working,

```bash
kubectl get pods --all-namespaces
```

## Common Operations

### See all nodes in cluster

```bash
kubectl get nodes
```

### See a specific node

```bash
kubectl describe node robotics-rover1
```

### See all deployed pods

- Notice that `-n clouddesktop-prod` refers to the clouddesktop-prod k8s `namespace`

```bash
kubectl -n clouddesktop-prod get pods
```

### See a specific pod

```bash
kubectl -n clouddesktop-prod describe pod julianho-clouddesktop-deployment-abc123efg-123abc
```

### Draining a node

```bash
kubectl drain robotics-rover2
```
