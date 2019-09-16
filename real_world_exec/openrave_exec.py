#!/usr/bin/env python
import sys,os
sys.path.append("/".join(os.path.abspath(os.path.abspath(__file__)).split('/')[:-2]))
from Simulators.OpenRaveSimulator import *
import pickle
import util
import Config
import openravepy

class arg():
	def __init__(self,type,value):
		self.type = type
		self.value = value

def correct_robot_pose(sim):
	joint_dump = open("init_joint_state.pickle","rb")
	joint_poses = pickle.load(joint_dump)
	joint_dump.close()
	robot = sim.env.GetRobot('fetch')
	robot_joints = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
					'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
	vals = [2,6,7,8,9,10,11,12]
	robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in robot_joints])
	robot.SetActiveDOFValues([v for i,v in enumerate(joint_poses[0]) if i in vals])

def convert_to_ros_traj(sim,type_exec,val,speed = 0.1):
	if type_exec == "GripperCloseTrajectory" or type_exec == "GripperOpenTrajectory":
		return val
	trajobj = openravepy.RaveCreateTrajectory(sim.env, '')
	openravepy.Trajectory.deserialize(trajobj, val)
	return util.ros_trajectory_from_openrave(sim.env.GetRobots()[0],trajobj, velocity_scale=speed)

if __name__ == '__main__':
	Config.setPaths()
	traj_dump = open("openrave_trajectories.pickle","rb")
	trajectory_list = pickle.load(traj_dump)
	traj_dump.close()

	sim = OpenRaveSimulator(Config.OPENRAVE_ENV_XML)
	correct_robot_pose(sim)
	ros_traj_list = []
	
	for traj in trajectory_list:
		e_type, e_val, e_all_generated = traj
		sim.execute(e_type,e_val,e_all_generated)
		ros_traj = convert_to_ros_traj(sim,e_type,e_val,speed=0.1)
		ros_traj_list.append([e_type,ros_traj])
		# raw_input()

	traj_dump = open("ros_trajectories.pickle","wb")
	pickle.dump(ros_traj_list, traj_dump)
	traj_dump.close()
	Config.resetPaths()
