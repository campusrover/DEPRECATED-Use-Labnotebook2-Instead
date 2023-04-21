# Container Image

Cloud Desktop Container uses a custom docker image. The `Dockerfile` is located [here](https://github.com/pitosalas/tb3-ros).

## Internals

### Components

There are 3 main components in the container image,

- VNC server paired with a NoVNC server
- VSCode server
- Tailscale client

### Defaults

Catkin Workspace: `/my_ros_data/catkin_ws`

Ports:

- `novnc` 80
- `vnc` 5900
- `vscode` 8080

### Layers

The current container image is structured this way:

![Layers](graphs/image-layer.svg)

`cosi119/tb3-ros`
  - Installs ROS melodic and ROS packages
  - Installs custom packages used in class, like `prrexamples`

`cosi119/ubuntu-desktop-lxde-vnc`
  - Provides a Ubuntu image with novnc and lxde preconfigured.
  - Provides a CUDA enabled variant (image with `-cuda` tag suffix)

## Process Management

### Supervisord

Each of the components are managed by a process control system called `supervisord`. Supervisor is responsible for spawning and restarting these components. For detailed configs, see [supervisord.conf](https://github.com/pitosalas/tb3-ros/blob/61c393140da2dbcff15fa48f0ba9c6435d5ff94c/tb3-ros/files/supervisor/supervisord.conf).

### Modifing startup processes

Modify the `supervisord.conf` under `tb3-ros/tb3-ros/files/supervisor/supervisord.conf`.

## Packages

### Default packages

As of version `2.1.1`,

- `turtlebot3_msgs`
- `turtlebot3`
- `turtlebot3_simulations`
- `https://github.com/campusrover/prrexamples`
- `https://github.com/campusrover/gpg_bran4`

### Adding a new package

To add a package to the default catkin workspace, modify the `Dockerfile` under `tb3-ros/tb3-ros/Dockerfile`:

```dockerfile
# Add the following lines
WORKDIR /my_ros_data/catkin_ws/src
RUN git clone --recursive --depth=1 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

## Github repo

[pitosalas/tb3-ros](https://github.com/pitosalas/tb3-ros)
