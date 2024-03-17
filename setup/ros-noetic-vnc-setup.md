## 0. Download Docker and WSL2
1. Install WSL2
    1. Open Microsoft Store
    2. Search wsl2
    3. Download Ubuntu Focal (The one listed as Ubuntu 20.04.6 LTS)
2. Install Docker Desktop: https://docs.docker.com/get-docker/

## 1. Download ROS Noetic VNC
NOTE: This is not the official ROS docker image, this is just a convenient image that comes with VNC and NoVNC capabilities: https://hub.docker.com/r/arvinskushwaha/ros-noetic-desktop-vnc
```
docker pull arvinskushwaha/ros-noetic-desktop-vnc
```

## 2. Create and Run
**Create**<br>
Create a container using this command, replace {CONTAINER_NAME} with your preferred name
```
docker run --name {CONTAINER_NAME} -p 5900:5900 -dit arvinskushwaha/ros-noetic-desktop-vnc bash
```
**I personally ran : docker run --name ros-noetic -p 5900:5900 -dit arvinskushwaha/ros-noetic-desktop-vnc bash* <br>
**Can also add -v flag to mount folders to the container to achieve file sharing between host and container*


**Run**<br>
To run the container, make sure the container listed in Docker GUI is started, then in command line run:
```
docker exec -it {CONTAINER_NAME} bash
```
In windows I recommend making a batch script to run the container, make a new file called **run-noetic.bat**, in the file write:
```bat
@echo off
docker exec -it {CONTAINER_NAME} bash
```
So running the container becomes simpler, simply double click on run-noetic.bat or in cmd navigate to it and run:
```cmd
ros-noetic.bat
```

**Login**<br>
When entering this container, the user is automatically root user. To login, use the script provided in root folder called login. In other words when terminal is first opened, run:
```bash
./login
```
**The login script is located in /root* <br>

**Source ROS**<br>
**No need to**, since already preconfigured .bashrc file to automatically run: source /opt/ros/noetic/setup.bash. As long as you logged in then it should work.<br><br>
To confirm ros is functional, run:
```bash
roscore
```
Result should be similar to:
```
... logging to /root/.ros/log/e15a1054-e34a-11ee-a2db-26dfeeba3fd7/roslaunch-docker-desktop-117.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://docker-desktop:32943/
ros_comm version 1.16.0


SUMMARY
========

PARAMETERS
* /rosdistro: noetic
* /rosversion: 1.16.0

NODES

auto-starting new master
process[master]: started with pid [125]
ROS_MASTER_URI=http://docker-desktop:11311/

setting /run_id to e15a1054-e34a-11ee-a2db-26dfeeba3fd7
process[rosout-1]: started with pid [135]
started core service [/rosout]
```

To exit the container, simply type (if you logged in "exit" will logout, then "exit" again to exit container):
```bash
exit
```

Usually recommended to update the package manager and upgrade the current packages
```
sudo apt-get update
sudo apt-get upgrade
```

## 3. Connect via VNC
Use any VNC viewer application.

Under Remote Host field, type: localhost:5900, and connect.
Or if it is separated into host and port, then host as localhost and port as 5900.