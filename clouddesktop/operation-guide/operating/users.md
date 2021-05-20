# User Management

## Requirements

- `kubectl` [Installation Guide](https://kubernetes.io/docs/tasks/tools/)
- [Setup Instructions](cluster.md#setup)
- `terraform` [Installation Guide](https://www.terraform.io/downloads.html)
- `make`
- AWS Credentials to AWS Route53
    - Access Key
    - Secret Key

## Setup

First obtain the users repo from [here](https://github.com/campusrover/clouddesktop-k8s).

Setup `.env` file by filling in all required fields:

```bash
mv .env.sample .env
```

It's better to use an IAM `user group` to create a new `user` associated with the `clouddesktop` user group. It will generate a access and secret key for you to put in the above file. The ingress IP is the ip address of the main node. Once everything is properly setup, do:

```bash
export $(make env)
```

Setup terraform:

```bash
terraform init
```

## Common Operations

To under what each of these commands do under the hood, see [here](../lifecycle.md).

### Add a new user

`id` is the ID of the new user.

```bash
make add-user id=example
```

### Delete a user

**Warning: This will remove any persisted data!!**

```bash
make delete-user id=example
```

### Change resources allocation for user

For user `example`, modify the file `example-clouddesktop/deployment.yaml`.

#### To increase minimum resource

*For detailed explanation of what units you can change it to, see [here](https://kubernetes.io/docs/concepts/configuration/manage-resources-containers/).*

```diff
resources:
  requests:  # increase minimum to at least 4 cores
-   cpu: 2
+   cpu: 4
    memory: 2Gi
```

#### To increase maximum resource limit

*For detailed explanation of what units you can change it to, see [here](https://kubernetes.io/docs/concepts/configuration/manage-resources-containers/).*

```diff
resources:
  limits: # increase to maximum 16GB of ram
    cpu: 8
-   memory: 8Gi
+   memory: 16Gi
```

#### Add GPU support

**Note: Beware if we have enough free GPUs in the cluster**

**Note: Make sure the docker image is a CUDA enabled variant (ie. `tb3-ros:v2.1.1-cuda`)**

```diff
resources:
  limits: # increase to maximum 16GB of ram
    cpu: 8
    memory: 8Gi
+   nvidia.com/gpu: 1
```

#### Apply changes

**Warning: This will restart the cloud desktop container!!**

To apply the previously changed values,

```bash
kubectl apply -k example-clouddesktop
```

### Restarting a Desktop

To restart a desktop, you need to delete and redeploy the desktop.

*This will NOT lead to loss of data.*

```bash
kubectl delete -k example-clouddesktop

kubectl apply -k example-clouddesktop
```
