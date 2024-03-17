## 1. Setting up ROS environment
**CODE WITHIN THIS SECTION SHOULD ONLY RUN ONCE** <br>
**For users that chose ros noetic image with VNC**<br>Navigate to ros_ws folder under user(/home/ubuntu/ros_ws)
```bash
cd ~/ros_ws/
```

Setup with catkin_make
```bash
catkin_make
```

**For users that chose official ros image**<br>
Run this(feel free to refer to the official [ROS guide](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)):
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
catkin_make
```
*NOTE*
- The ros_ws parent folder is the ROS workspace folder
- src folder under it is where you place ROS packages


## 2. Clone INSERR-ROS
**CODE WITHIN THIS SECTION SHOULD ONLY RUN ONCE** <br>
Right now there are no packages installed <br>
Clone the INSERR-ROS package from github
```bash
cd src
git clone https://github.com/RickyVu/INSERR-ROS.git
# Cloning into 'INSERR-ROS'...
# remote: Enumerating objects: 25, done.
# remote: Counting objects: 100% (25/25), done.
# remote: Compressing objects: 100% (19/19), done.
# remote: Total 25 (delta 7), reused 22 (delta 5), pack-reused 0
# Unpacking objects: 100% (25/25), 9.88 KiB | 98.00 KiB/s, done.
```

Don't forget to install all the required python libraries
```bash
cd INSERR-ROS
pip3 install -r requirements.txt
```
## 3. INSERR-ROS guide
This INSERR-ROS directory is a ROS package and also a git repository


Most important files are these:
```bash
ros_ws/
|-- build
|-- devel
|   |-- setup.bash
`-- src
    `-- INSERR-ROS
        |-- CMakeLists.txt
        |-- README.md
        |-- package.xml
        |-- scripts
        |   |-- CAN_Handler.py
        |   `-- pubsub
        |       `-- json_message.py
        |-- setup
        |   |-- ros-noetic-setup.md
        |   `-- ros-noetic-vnc-setup.md
        `-- setup.py
```

**Source setup.bash**<br>
Need to run this command for ros commands to work in command line (Run this for every new terminal)
```bash
source ~/ros_ws/devel/setup.bash
```

Program files are placed in scripts folder, classes and functions that can be imported should be placed in a subfolder under scripts. (Like the pubsub folder shown above)

## 4. Source Control
When using VS Code Source Control, by default it will say there are no git repositories initialized. Need to first run:
```bash
git config --global --add safe.directory ~/ros_ws/src/INSERR-ROS
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
```

Also switch to another branch before developing features
```bash
git checkout dev
```
Or make a new branch
```bash
git checkout -b {branch-name}