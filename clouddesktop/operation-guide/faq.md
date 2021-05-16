# FAQ

## User

### How do I add a new user?

Read [User Management](operating/users.md).

### How do I delete a user?

Read [User Management](operating/users.md).

### How do I upgrade/downgrade a desktop?

Read [User Management](operating/users.md).

### How do I see all the active desktops in the cluster?

```bash
kubectl -n clouddesktop-prod get pods
```

## Image

### How do I add a package to the image?

Read [Adding a new package](image.md#adding-a-new-package).

To make sure that new desktops are receiving this update, modify the deployment template under [`clouddesktop-k8s/clouddesktop-template/deployment.yaml`](https://github.com/campusrover/clouddesktop-k8s/blob/1b9cb74bd6646bcbd15813e8352d45df2b5388be/clouddesktop-template/deployment.yaml).

```diff
spec:
  initContainers:
    - name: init-clouddesktop
-     image: cosi119/tb3-ros:v2.1.1
+     image: cosi119/tb3-ros:v2.1.2
```

## Node

### Is it safe to restart a node?

Yes, but make sure to drain the node first. To drain the node,

```bash
kubectl drain robotics-rover2
```

After the reboot, `k3s` will be started automatically. If it is not started,

```bash
sudo systemctl restart k3s.service
```

## Troubleshooting

### Desktop connectivity issue

If a group or all desktops are not connecting,

![Debug Group](graphs/debug-connection-group.svg)

If only 1 desktop is not connecting,

![Debug one](graphs/debug-connection-one.svg)
