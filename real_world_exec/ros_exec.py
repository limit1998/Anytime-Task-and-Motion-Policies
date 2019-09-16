#!/usr/bin/env python
import actionlib
from fetch_control import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, JointTolerance, FollowJointTrajectoryAction
import rospy
import copy
import time
import pickle
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# import moveit_commander
# import tf2_geometry_msgs
# import tf2_ros
import time

def execute_on_ros(ros_traj):
    client = actionlib.SimpleActionClient('/arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    id_gen = actionlib.GoalIDGenerator()
    jtg = FollowJointTrajectoryGoal()
    jtg.trajectory = ros_traj
    client.send_goal(jtg)
    client.wait_for_result()
    time.sleep(3)

# Do not use this in conjunction with ros trajectories generated with openrave
# def pre_grasp_to_grasp(group):
# 	tfBuffer = tf2_ros.Buffer()
# 	listener = tf2_ros.TransformListener(tfBuffer)
# 	while not rospy.is_shutdown():
# 		try:
# 			tstmp = rospy.Time()
# 			trans1 = tfBuffer.lookup_transform('wrist_roll_link','base_link', tstmp)
# 			break
# 		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
# 			print("Fatal error!")
# 			time.sleep(0.1)
# 	print("trans1", trans1)
# 	while not rospy.is_shutdown():
# 		try:
# 			tstmp = rospy.Time()
# 			trans2 = tfBuffer.lookup_transform('base_link','wrist_roll_link', tstmp)
# 			break
# 		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
# 			print("Fatal error!")
# 			time.sleep(0.1)
# 	print("trans2", trans2)
	
# 	waypoints = []
# 	# start with the current pose
# 	print("Prefail")
# 	while not rospy.is_shutdown():
# 		try:
# 			cp = group.get_current_pose().pose
# 			break
# 		except:
# 			print("Fatal error!")
# 			time.sleep(0.1)
# 	print("Current pose: ",cp)
# 	waypoints.append(cp)

# 	wpose = Pose()
# 	wpose.position.x = waypoints[0].position.x
# 	wpose.position.y = waypoints[0].position.y
# 	wpose.position.z = waypoints[0].position.z
# 	wpose.orientation.w = waypoints[0].orientation.w
# 	wpose.orientation.x = waypoints[0].orientation.x
# 	wpose.orientation.y = waypoints[0].orientation.y
# 	wpose.orientation.z = waypoints[0].orientation.z

# 	#Transform pose in base frame to gripper frame
# 	base_pose_stamped = PoseStamped()
# 	base_pose_stamped.header.frame_id = 'base_link'
# 	base_pose_stamped.header.stamp = rospy.Time.now()
# 	base_pose_stamped.pose = wpose
# 	pose_in_gripper_frame = tf2_geometry_msgs.do_transform_pose(base_pose_stamped, trans1)
# 	wpose = pose_in_gripper_frame.pose

# 	#Offset in x axis of gripper frame
# 	wpose.position.x = 0.2

# 	#Transform pose in base frame to gripper frame
# 	gripper_pose_stamped = PoseStamped()
# 	gripper_pose_stamped.header.frame_id = 'base_link'
# 	gripper_pose_stamped.header.stamp = rospy.Time.now()
# 	gripper_pose_stamped.pose = wpose

# 	grasp_pose_in_base_frame = tf2_geometry_msgs.do_transform_pose(gripper_pose_stamped, trans2)
# 	wpose = grasp_pose_in_base_frame.pose
# 	waypoints.append(copy.deepcopy(wpose))
# 	(plan, fraction) = group.compute_cartesian_path(
#                        waypoints,   # waypoints to follow
#                        0.001,        # eef_step
#                        0.0)         # jump_threshold
# 	print("Execute Grasp")
# 	group.execute(plan, wait=True)

if __name__ == '__main__':
	rospy.init_node("physical_fetch_execution")
	gripper = Gripper()
	# group = moveit_commander.MoveGroupCommander("arm_with_torso")
	traj_dump = open("ros_trajectories.pickle","rb")
	trajectory_list = pickle.load(traj_dump)
	traj_dump.close()
	gripper.open()
	
	print("Press Enter To Start execution")
	raw_input()
	for traj in trajectory_list:
		e_type, e_val = traj
		if e_type == "GripperCloseTrajectory":
			gripper.close()
		elif e_type == "GripperOpenTrajectory":
			gripper.open()
		elif e_type == "ManipTrajectory" or e_type == "PGManipTrajectory":
			execute_on_ros(e_val)
		else:
			print("Unknown Type. Exit code")
			break
			continue
		print("Press Enter For Next Step")
		raw_input()
