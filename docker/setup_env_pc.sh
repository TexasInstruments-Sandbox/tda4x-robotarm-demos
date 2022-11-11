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

if [ "${BASH_SOURCE[0]}" -ef "$0" ]; then
    echo "You should source this script: source ${BASH_SOURCE[0]}"
    exit 1
fi

# NOTE: Update the following lines based on your setting
export J7_IP_ADDR=128.247.79.94
export PC_IP_ADDR=128.247.79.240
export ROS_WS=${HOME}/j7ros_home/ros_ws

# Source setup.bash in case where ROS is installed on the host Ubuntu
if [[ ! -z "${ROS_VERSION+x}" ]]; then
    if [[ "$ROS_VERSION" == "1" ]]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
        # ROS 1 network setting
        export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
        export ROS_IP=$PC_IP_ADDR
        # source ros_ws setup.bash if exists
        SETUP_FILE=$ROS_WS/devel/setup.bash
        if [ -f $SETUP_FILE ]; then
            source $SETUP_FILE
        fi
    elif [[ "$ROS_VERSION" == "2" ]]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
        # source ros_ws setup.bash if exists
        SETUP_FILE=$ROS_WS/install/setup.bash
        if [ -f $SETUP_FILE ]; then
            source $SETUP_FILE
        fi
    else
        echo "ROS is not set up or invalid ROS_VERSION"
    fi
fi
