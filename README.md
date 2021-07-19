# Developing C++ ROS in CLion

This is an example project to illustrate how use CLion as a fully integrated development
environment for a ROS C++ project by encapsulating the dependencies in a docker container.

## The turtle example

This example installs dependencies for the [`turtlesim`](http://wiki.ros.org/turtlesim) package on top of the base ROS image.
It defines a new package called [`turtle_example`](./turtle_example) with a simple C++ node called [`circler.cpp`](./turtle_example/src/circler.cpp)
which publishes twist commands for a running turtlesim node to drive it in a circle.

## Base ROS Docker images

The majority of the background work is done in the repository [aica-technology/docker-images](https://github.com/aica-technology/docker-images).
There, docker images are defined for specific versions of ROS and ROS2 that contain
all of the necessary development dependencies, including an inbuilt SSH server.
CLion will use the SSH server to access the remote environment.

To begin, clone the [docker-images](https://github.com/aica-technology/docker-images) repository and follow the instructions to pull or build your
base ROS workspace of choice.

## CLion setup

Prerequisites: Docker, CLion 2020 or newer, base ROS docker image from AICA

Step 1: Build the docker image with `docker build -t eeberhard/ros-clion-example .` (this will take a while the first time around).

Step 2: Use the [`server.sh` script](https://github.com/aica-technology/docker-images/blob/master/scripts/server.sh) in
the aica-technology/docker-images repository to start a background container for the appropriate user
with a desired port number.

In this example, we will use user `ros` for a noetic version image. For ROS2 versions, use user `ros2`.
We will also choose to use port 3456, though the specific number does not matter.

```shell script
/.../aica-technology/docker-images/scripts/server.sh eeberhard/ros-clion-example --user ros --port 3456
```

Step 3: If not already done, [create a remote toolchain in CLion](https://www.jetbrains.com/help/clion/remote-projects-support.html#remote-toolchain)
using the following credentials:
 - host `127.0.0.1`
 - port `3456` (or desired port number)
 - Authentication type: Key Pair
 
Step 4: If not already done, [create a CMake profile that uses the remote toolchain](https://www.jetbrains.com/help/clion/remote-projects-support.html#CMakeProfile).

Step 5: Optional but recommended: [set the deployment configuration](https://www.jetbrains.com/help/clion/remote-projects-support.html#deployment-entry)
to synchronise local file changes between the host and the container. 
For example, by mapping the local path [`turtle_example`](./turtle_example) to the remote path `/home/ros/ros_ws/src/turtle_example`,
your local changes will automatically update the package in the remote workspace.

Step 6: Press apply and load the CMakeLists.txt of your package. After a period of loading the cmake project
and indexing the files, your IDE should be fully configured for the project!

### Troubleshooting:

- If you have issues with finding files or building the project, try [resyncing the header search paths](https://www.jetbrains.com/help/clion/remote-projects-support.html#resync).
Repeat this step any time you change the compiler or project dependencies.
Optionally you can enable automatic synchronization.

- If you have cmake errors, check that you have [selected the remote CMake profile](https://www.jetbrains.com/help/clion/remote-projects-support.html#WorkWithRemote)
to build, run and debug entirely with the remote toolchain.

- If you cannot connect to the remote container through CLion, check that you can do so from the command
line (for example `ssh ros@127.0.0.1 -p3456`), and that you have the correct access credentials in the configuration. 


## Using the example

Once the server is running in the background, you can use the [`connect.sh` script](https://github.com/aica-technology/docker-images/blob/master/scripts/connect.sh) 
to attach new terminal windows by specifying the container name and user. You can use `docker container ls` to 
lookup the names of running containers. In the case of this example:

```shell script
/.../aica-technology/docker-images/scripts/connect.sh eeberhard-ros-clion-example-ssh --user ros
```

Using three terminals, invoke `roscore` in one, then `rosrun turtlesim turtlesim_node` in a second, and finally
`catkin_make && rosrun turtle_example circler` in the third. You should see the turtle begin to swim in circles.

With the CLion environment properly established, you will also be able to build, run and debug the `circler` target
from within the IDE instead of from the third terminal and still see the effect on the turtlesim window.

Note that for GUI applications through Docker on MacOS, you will need to follow the additional 
[display forwarding instructions](https://github.com/aica-technology/docker-images#notes-on-x11-display-forwarding-for-mac) in the docker-images repository.