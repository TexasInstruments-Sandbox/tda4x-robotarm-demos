# Robot Arm Demos on TI Platform

This repository maintains demos targeted for the Niryo Ned2 Robot Arm with TI TDA4VM platforms. This document walks through how to run the applications provided in this repository.


## Package Components

This repository contains the following:

1. The `nodes` sub-directory contains the Python modules that implements a robot arm following demo.
2. The `docker` sub-directory contains docker file to build docker container
3. The `patches` sub-directory contains GIT patches for edge_ai_apps and [Niryo ned_ros](https://github.com/NiryoRobotics/ned_ros) repositories needed to run the robot arm following demo. 
4. The `scripts` sub-directory includes scripts needed to set up demos, e.g. applying the GIT patches to the corresponding repos.


## Environnment Setup 

### Hardware Components 

The following hardware components are necessary for robot arm demos:

1. A [Niryo Ned2](https://niryo.com/) robot arm. It would be possible to use Niryo Simulator instead. But this document does not explain how to run the demo using the Niryo simulator. 
2. A working [TDA4VM SK board](https://www.ti.com/tool/SK-TDA4VM), E2 revision or higher and the SD card flashed with the latest stable EdgeAI SDK with Robotics SDK.
3. [IWR6843](https://www.ti.com/product/IWR6843) radar sensor connected to the SK board through USB.
4. A USB camera connected to the SK board.
5. Niryo and SK board should be connected to the same network through either Ethernet router or switch.
6. A monitor to display camera input and detection output overlaid on top of it.


### Setting Up J7 SK Board

Download the prebuilt SD card image and flash it to a SD card by following the [Edge AI Documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_04_00/exports/docs/getting_started.html).

Follow the instructions in the Robotics SDK User Guide Documentation on  [Setting up Robotics SDK](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_04_00/docs/source/docker/README.html#setting-up-robotics-kit-environment) and [Docker Setup for ROS1](hhttps://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_04_00/docs/source/docker/setting_docker_ros1.html) to install Robotics SDK and work with ROS1 on J7 SK board.


1. Once logged on to the SK board, clone the niryo-tda4vm repository under the j7_ros_home directory.

```
root@tda4vm-sk:/opt/edge_ai_apps# cd ..
root@tda4vm-sk:/opt# mkdir robot && cd robot
root@tda4vm-sk:/opt/robot# git clone --single-branch --branch master https://github.com/TexasInstruments/tda4x-robotarm-demos.git
```


2. Run `init_setup.sh`.

```
root@tda4vm-sk:/opt/robot# cd tda4x-robotarm-demos
root@tda4vm-sk:/opt/robot# source scripts/init_setup.sh
```

This script performs
- Apply a patch (edge_ai_apps_for_robot_arm.patch) to /opt/edge_ai_apps.
- Clone the Niryo ned_ros GIt repo and apply a patch (ned_ros_tda4.patch).
- Set up the soft links under `/opt/robot/tda4x-robotarm-demos/nodes/arm_follow_demo/robot_arm_follow_demo` to the package and messages in the ned_ros GIT repo. They are needed by the [Robot Arm Following]>(nodes/arm_follow_demo/robot_arm_follow_demo/README.md) demo. 
- Install the mmWave radar driver ROS node.
- Build Melodic docker container to run Moveit package on the SK board. 

If the above commands are successful, the directory structure should look as follows:

```
/opt/robot/tda4x-robotarm-demos# 
+ README.md
+ docker/
+ nodes/
  + arm_follow_demo/
    + niryo_robot_arm_commander/
    + niryo_robot_msgs/
    + robot_arm_follow_demo/
  + ned_ros/
  + radar_driver
    + ti_mmwave_rospkg/
    + serial/
  + patches/
  + scripts/
```

And you can check if docker images are built successfully. An example output based on 8.4 release is shown below.

```
root@tda4vm-sk:~/j7ros_home# docker images

REPOSITORY                                                   TAG       IMAGE ID            CREATED       SIZE
registry.gitlab.com/niryo/niryo-one-s/ned_ros_stack/v4.1.0   arm64                eed165f6d06b        2 minutes ago       2.84GB
ros                                                          melodic-perception   eec268b440d3        19 minutes ago      2.23GB
j7-ros-noetic                                                8.4                  f6c9f4f93ba8        48 minutes ago      3.4GB
artifactory.itg.ti.com/docker-public-arm/ubuntu              18.04                5777305f3710        16 months ago       56.6MB
artifactory.itg.ti.com/docker-public-arm/ubuntu              20.04                08f5c0d9d654        17 months ago       65.5MB```
```

3. Finally, we should update `~/j7ros_home/docker_run_ros1.sh` for volume mapping of the newly created `/opt/robot` directory in the Noetic Docker container. So add `-v /opt/robot:/opt/robot` as a parameter to `docker run` as follows:

```
docker run -it --rm \
    -v /opt/robot:/opt/robot \
    -v /home/root/j7ros_home:/root/j7ros_home \
    -v /opt/robotics_sdk:/opt/robotics_sdk \
    -v /home/root/j7ros_home/.ros:/root/.ros \
```

### Setting Up Niryo Robot Arm

Niryo robot arm has Raspberry Pi 4 with Ubuntu 18.04, which is ROS Melodic is running on. Therefore, there is any additional set up needed except for disabling Moveit since Moveit is running on the SK board. Once logged on to the Niryo robot arm, we only need to comment out move_group.launch from ~/catkin_ws/src/niryo_robot_arm_commander/launch/robot_commander.launch, e.g.

```xml
....
 
<include file="$(find niryo_robot_arm_commander)/launch/robot_commander_base.launch.xml">
<arg name="hardware_version" value="$(arg hardware_version)" />
</include>
 
<!-- Moveit move_group -->
<!--
<group unless="$(arg gazebo)">
<include file="$(find niryo_moveit_config_standalone)/launch/move_group.launch">
<arg name="hardware_version" value="$(arg hardware_version)" />
<arg name="simulation_mode" value="$(arg simulation_mode)" />
</include>
</group>
 
<group if="$(arg gazebo)">
<include file="$(find niryo_moveit_config_w_gripper1)/launch/move_group.launch">
<arg name="load_robot_description" value="false"/>
<arg name="hardware_version" value="$(arg hardware_version)" />
<arg name="simulation_mode" value="$(arg simulation_mode)" />
</include>
</group>
-->

...
```

### Setting Up Ubuntu PC

Please follow the instructions in the Robotics SDK User Guide Documentation on [Set Up Docker Environment on the Remote PC for Visualization](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_04_00/docs/source/docker/setting_docker_ros1.html#set-up-docker-environment-on-the-remote-pc-for-visualization).

This is optional though since a current robot arm demo does not require visualization on Ubunt PC.


## Running Demos

Please follow the link below for details on running the robot arm demo.

[Robot Arm Following Demo](nodes/arm_follow_demo/robot_arm_follow_demo/README.md)

<!--
# Robot Arm Follow Demo

## Demo Setup

### Required Software

#### SK Board

* Edge AI SDK
  * Edited edge_ai_apps/apps_python/ repository

* Robotics SDK
  * ROS1 docker container
  * ROS1 packages:
    * roscpp
    * rospy
    * geometry_msgs
    * shape_msgs
    * std_msg
    * niryo_robot_arm_commander
    * niryo_robot_msgs

#### External PC (for Simulation)

* Ubuntu 18.04
* ROS Melodic
* Niryo's [ned_ros stack](https://github.com/NiryoRobotics/ned_ros)

### Required Hardware

* TDA4VM SK Board
* USB camera
* Robotic arm:
  * Simulation: Ubuntu 18.04 machine
  * Real demo: Niryo Ned2 robotic arm

### Installation Instructions:

#### Ubuntu Machine (simulation)
1. Create a ROS workspace:
  ```
  $ mkdir -p niryo_ws/src
  ```

2. Clone the `ned_ros` repository:
  ```
  $ cd niryo_ws
  $ git clone https://github.com/NiryoRobotics/ned_ros src
  ```

3. Build the ROS project:
  ```
  $ pip2 install -r src/requirements_ned2.txt
  $ rosdep update
  $ rosdep install --from-paths src --ignore-src --default-yes --rosdistro melodic --skip-keys "python-rpi.gpio"
  $ source /opt/ros/melodic/setup.bash
  $ catkin_make
  ```

#### Niryo Ned2 (real)

1. Nothing

#### SK Board

1. Follow the instructions to [install the Robotics SDK](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/README.html) for ROS1

2. Start the docker container:
  ```
  $ cd ~/j7ros_home
  $ ./docker_run_ros1.sh
  ```

3. Clone the human_pose demo and the edited edge_ai_apps repositories:
  ```
  $ git clone --single-branch --branch master https://bitbucket.itg.ti.com/scm/pal_ros/niryo-tda4vm.git
  $ git clone --single-branch --branch robotic_arm_demo https://bitbucket.itg.ti.com/scm/processor-sdk-vision/edge_ai_apps.git
  ```

4. Build the ROS workspace:
  ```
  $ cd ~/j7ros_home/niryo-tda4vm/ros_ws
  $ catkin_make
  ```

## Demo Execution

### Ubuntu Machine (simulation)

1. Set the `ROS_IP` to the machine's ip address:
  ```
  $ ifconfig # to find the ip address
  $ export ROS_IP=ip_address
  ```

2. Source the ROS project:
  ```
  $ source devel/setup.bash
  ```

3. Run the simulation:
  ```
  $ roslaunch niryo_robot_bringup desktop_rviz_simulation.launch
  ```

### Niryo Ned2 (real)

1. Kill the ROS master:
  ```
  $ killall -9 rosmaster
  ```

2. Set the `ROS_IP` to the machine's ip address:
  ```
  $ ifconfig # to find the ip address
  $ export ROS_IP=ip_address
  ```

3. Source the ROS project:
  ```
  $ source catkin_ws/install/release/ned2/setup.bash
  ```

4. Start the robotic arm:
  ```
  $ roslaunch niryo_robot_bringup niryo_ned2_robot.launch
  ```

### SK Board

1. Set the `$ROS_MASTER_URI` to the Ubuntu machine or Ned2's ROS master:
  ```
  $ export ROS_MASTER_URI=http://ned2_ip_address:11311
  ```

Run both the edge_ai_apps script and the `robot_arm_follow` node: either use tmux to open two terminals within the same session or connect to the robot within two terminals don't forget to source the ROS projects before running the nodes.

2. Within one terminal, run the edge_ai_apps script for human pose estimation model:
  ```sh
  $ source ~/j7ros_home/niryo-tda4vm/ros_ws/devel/setup.bash
  $ cd ~/j7ros_home/edge_ai_apps/apps_python
  $ ./app_edgeai.py ../configs/human_pose_estimation.yaml
  ```

3. Within a second terminal, run the `robot_arm_follow` node:
  ```sh
  $ source ~/j7ros_home/niryo-tda4vm/ros_ws/devel/setup.bash
  $ rosrun robot_arm_follow_demo robot_arm_follow.py
  ```

TODO:
* add quality of life edits to the human_pose scripts
* combine the human_pose scripts into a ROS package and write a launch file

-->