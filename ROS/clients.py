# Move base using navigation stack

import actionlib
import rospy
import util
import openravepy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from control_msgs.msg import GripperCommandAction


#export ROS_MASTER_URI=http://192.168.1.2:11311

TYPE_QUAT_AND_TRANS = "type_quat_and_trans"
TYPE_TRAJ = "type_trajectory"
TYPE_GRIPPER_ACTION = "type_gripper_action"
ACTION = 'gripper_controller/gripper_action'
CLOSE_POS = 0.0  #meters
OPEN_POS = 0.10 #meters
DEFAULT_MAX_EFFORT = 50


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, ori_z, ori_w, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y

        #move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        #move_goal.target_pose.pose.orientation.w = cos(theta/2.0)

        move_goal.target_pose.pose.orientation.z = ori_z
        move_goal.target_pose.pose.orientation.w = ori_w
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


class FetchGripperClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(ACTION,GripperCommandAction)
        self.client.wait_for_server()


    def open(self,max_effort=None,position=None,):
        """
        max_effort: in Newton
        """
        if(not position):
            position = OPEN_POS
        if(not max_effort):
            max_effort = DEFAULT_MAX_EFFORT

        self.actuateGripper(position,max_effort)

    def close(self,max_effort=None,position=None,):
        """
        max_effort: in Newton, ensure > 35 to close
        """
        if(not position):
            position = CLOSE_POS
        if(not max_effort):
            max_effort = DEFAULT_MAX_EFFORT

        self.actuateGripper(position,max_effort)


    def actuateGripper(self,pos,max_eff):
        goal = GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = DEFAULT_MAX_EFFORT
        if(max_eff):
            goal.command.max_effort = max_eff
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(5.0))

        result = self.client.get_result()

        if(not result):
            print "ERROR: Gripper actuate result returned None, "
        else:
            return result
