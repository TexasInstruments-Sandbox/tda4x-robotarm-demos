#!/bin/bash

#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

export PROJ_PATH=/opt/robot/tda4x-robotarm-demos

# 1. Apply a patch to /opt/edge_ai_apps
echo "Applying a patch to /opt/edge_ai_apps ..."
bash $PROJ_PATH/scripts/setup_edge_ai_apps.sh

# 2. Clone the Niryo ned_ros repository and apply a  patch
echo "Cloning Niryo ned_ros and applying a patch ..."
bash $PROJ_PATH/scripts/setup_ned_ros.sh

# 3. Set softlink to Niryo package and messages for robot_arm_follow_demo
echo "Setting softlinks to Niryo package and messages for robot_arm_follow_demo ..."
bash $PROJ_PATH/scripts/setup_tda4vm_with_ned.sh

# 4. Install the mmWave radar driver ROS node
echo "Installing the mmWave radar driver ROS node ..."
bash $PROJ_PATH/scripts/setup_radar_driver.sh

# 5. Build Melodic docker conatiner to run Moveit package on the SK board
echo "Building Melodic docker container ..."
bash $PROJ_PATH/scripts/setup_docker_melodic.sh