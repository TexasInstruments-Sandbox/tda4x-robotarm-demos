#!/usr/bin/env python3

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

"""
File: robot_arm_follow.py
Author: Jared McArthur
Description: holds the ROS code to execute ArmCommands based on the human poses given from the /human_pose node.
"""

import numpy as np
import rospy
import actionlib
from geometry_msgs.msg import Point
from shape_msgs.msg import SolidPrimitive
from niryo_robot_msgs.msg import RPY
from niryo_robot_arm_commander.msg import ArmMoveCommand, RobotMoveAction, RobotMoveGoal
from robot_arm_follow_demo.msg import Frame
from robot_arm_follow_demo.msg import HumanPose
from robot_arm_follow_demo.msg import RadarOccupancy

class HumanPoseSubscriber:
    __subscriber: rospy.Subscriber
    __timer: rospy.Timer
    __action_client: actionlib.SimpleActionClient
    __human_pose: HumanPose

    def __init__(self) -> None:
        # set parameters
        rospy.set_param("~robot_reach/dimensions/x", 0)
        rospy.set_param("~robot_reach/dimensions/y", 0.4)
        rospy.set_param("~robot_reach/dimensions/z", 0.34)
        rospy.set_param("~robot_reach/center_point/x", 0.2)
        rospy.set_param("~robot_reach/center_point/y", 0.0)
        rospy.set_param("~robot_reach/center_point/z", 0.25)
        rospy.set_param("~arm_command_frequency", 0.05)

        # create the subscriber, timer, and action client
        self.__subscriber = rospy.Subscriber("human_pose", HumanPose, self.subscriber_callback)
        self.__subscriber_radar = rospy.Subscriber("/ti_mmwave/radar_occupancy", RadarOccupancy, self.subscriber_radar_callback)
        self.__action_client = actionlib.SimpleActionClient("niryo_robot_arm_commander/robot_action", RobotMoveAction)
        self.__action_client.wait_for_server()
        self.__action_client.cancel_all_goals()
        self.__action_client.wait_for_result()
        self.__human_pose = None
        self.__radar_occupancy = 0
        self.__timer = rospy.Timer(rospy.Duration(rospy.get_param("~arm_command_frequency")), self.timer_callback)

    def subscriber_callback(self, human_pose: HumanPose) -> None:
        self.__human_pose = human_pose
    
    def subscriber_radar_callback(self, radar_occupancy : RadarOccupancy) -> None:
        self.__radar_occupancy = radar_occupancy.state 
        print("radar_occupancy:%d" % self.__radar_occupancy)

    # TODO: timer method has essentially just added latency, can remove
    def timer_callback(self, event) -> None:
        if self.__human_pose != None and self.__radar_occupancy == 0:
            # input new command if arm is done moving:
            # possible states:
            #   PENDING=0
            #   ACTIVE=1
            #   PREEMPTED=2
            #   SUCCEEDED=3
            #   ABORTED=4
            #   REJECTED=5
            #   PREEMPTING=6
            #   RECALLING=7
            #   RECALLED=8
            #   LOST=9
            if self.__action_client.get_state() in [3, 4, 5, 8, 9]:
                # get parameters 
                x_dimension = rospy.get_param("~robot_reach/dimensions/x")
                y_dimension = rospy.get_param("~robot_reach/dimensions/y")
                z_dimension = rospy.get_param("~robot_reach/dimensions/z")
                x_center_point = rospy.get_param("~robot_reach/center_point/x")
                y_center_point = rospy.get_param("~robot_reach/center_point/y")
                z_center_point = rospy.get_param("~robot_reach/center_point/z")

                # get robot reach values
                robot_reach_y_min = y_center_point - y_dimension / 2
                robot_reach_y_max = y_center_point + y_dimension / 2
                robot_reach_z_min = z_center_point - z_dimension / 2
                robot_reach_z_max = z_center_point + z_dimension / 2

                # get human pose and frame values
                # use the left wrist since the input has been mirrored
                right_wrist = self.__human_pose.left_wrist
                frame = self.__human_pose.frame
                frame_y_min = frame.center_point.y - frame.box.dimensions[SolidPrimitive.BOX_Y] / 2
                frame_y_max = frame.center_point.y + frame.box.dimensions[SolidPrimitive.BOX_Y] / 2
                frame_z_min = frame.center_point.z - frame.box.dimensions[SolidPrimitive.BOX_Z] / 2
                frame_z_max = frame.center_point.z + frame.box.dimensions[SolidPrimitive.BOX_Z] / 2

                # find the x ,y, and z
                x = x_center_point
                y = robot_reach_y_min + (right_wrist.y - frame_y_min) * y_dimension / frame.box.dimensions[SolidPrimitive.BOX_Y] 
                z = robot_reach_z_min + (right_wrist.z - frame_z_min) * z_dimension / frame.box.dimensions[SolidPrimitive.BOX_Z] 

                # find the roll, pitch, and yaw
                roll = 0.0
                pitch = 0.0
                yaw = np.arctan2(y, x)

                # send goal
                goal = RobotMoveGoal(cmd=ArmMoveCommand(cmd_type=ArmMoveCommand.POSE, position=Point(x, y, z), rpy=RPY(roll, pitch, yaw)))
                self.__action_client.send_goal(goal)

                if not self.__action_client.wait_for_result(timeout=rospy.Duration(2.5)):
                    self.__action_client.cancel_goal()
                    self.__action_client.stop_tracking_goal()

if __name__ == "__main__":
    rospy.init_node("human_pose_subscriber")
    node = HumanPoseSubscriber()
    rospy.spin()
