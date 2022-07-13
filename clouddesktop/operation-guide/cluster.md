# Kubernetes Cluster

Cloud Desktop cluster is managed by Kubernetes. Specifically, it is a distro of Kubernetes named [`K3s`](https://rancher.com/docs/k3s/latest/en/).

## Management

For cluster management, see [Cluster Management](operating/cluster.md).

## Configurations

We use [`k3s`](https://rancher.com/docs/k3s/latest/en/architecture/) to run our k8s cluster. While leaving most configurations as default, we made the following customization,

- container runtime: `docker`

![K3s Architecture](https://rancher.com/docs/img/rancher/k3s-architecture-single-server.png)

## Namespaces

- `clouddesktop-prod`
  - Main namespace
  - Used for all active cloud desktops
- `clouddesktop-dev`
  - For testing cloud desktop images

## Networking

The default networking backend is `flannel` with VXLAN as backend.

## Ingress

The default ingress controller is `traefik`.

## Storage

The default storage provider is `local-path`. In other words, we store all cloud desktop files locally on the node.

## Node roles

*As of May 2021*

| Name | Role |
| --- | --- |
| robotics-rover1 | Master |
| robotics-rover2 | Agent |
