# bru.py - the script

## Installation

The bru.py script can be found in the [rosutils](https://github.com/campusrover/rosutils) repository. All our robots and VMs have this directory. However on a new robot, we create a symbolic link accessible to the path that is called just `bru` so the commands are `bru help` etc.

## Configuration

Whenever a new robot or robot type is created, certain key lines of bru.py need to be updated. Also of course, as we develop or fix bugs in the BRU package, that code will need to ne revised from time to time.

## Installation

In order to make it convenient to use BRU as a cli:

1. Create a symbolic link to the bru.py package.

`ln -s /my_ros_data/rosutils/bru.py /usr/local/bin/bru`

2. Make it executable

`chmod +x /my_ros_data/rosutils/bru.py`

