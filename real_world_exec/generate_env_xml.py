import sys, os
sys.path.append("/".join(os.path.abspath(os.path.abspath(__file__)).split('/')[:-2]))
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import JointState
import rospy
import roslaunch
import time
import pickle
import Config

def exec_launch(string):
	uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
	roslaunch.configure_logging(uuid)
	launch = roslaunch.parent.ROSLaunchParent(uuid, [string])
	launch.start()
	return launch

def robot_joint_state_extractor():
	def get_pose_message():
		while(1):
			try:
				msg = rospy.wait_for_message("joint_states", JointState)
				break
			except:
				print("Fatal error in joint pose estimation!")
				time.sleep(1)
				continue
		return msg

	joint_dump = open(Config.PROJ_DIR+"real_world_exec/init_joint_state.pickle","wb")
	while(1):
		msg = get_pose_message()
		if len(msg.name) == 13:
			print(msg.name)
			print(msg.position)
			break
	pickle.dump([msg.position], joint_dump)
	joint_dump.close()
	print("...Joint Joint State Extracted...")
	print(msg.position)


def get_ar_message():
	while(1):
		try:
			msg = rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
			break
		except:
			print("Fatal error!")
			time.sleep(1)
			continue
	return msg

def table_pose_extractor():
	# Get Table Info
	launch = exec_launch(Config.PROJ_DIR+"real_world_exec/fetch_multi_markers_black_table.launch")
	while(1):
		msg = get_ar_message()
		for i in msg.markers:
			if(i.id == 20):
				table_id = i.id
				table_pose = i.pose.pose
				break
		try:
			if table_pose.position.x > 0.1:
				break
		except:
			time.sleep(2)
	
	print("...Table Pose Extracted...")
	print(table_pose)
	launch.shutdown()
	return table_pose

def cylinder_pose_extractor(marker_ids):
	# a = input("Press Enter ONLY After runnning the following in mpm@fetch51: roslaunch ar_track_alvar fetch_indiv_can.launch")
	print("Tilt Head Around If Necessary to capture all markers")
	launch = exec_launch(Config.PROJ_DIR+"real_world_exec/fetch_indiv.launch")
	cylinder_poses = []
	for ar_id in marker_ids:
		print("Searching for cylinder", ar_id - marker_ids[0], " with ar_id:", ar_id)
		while(1):
			msg = get_ar_message()
			for i in msg.markers:
				if(i.id == ar_id):
					cylinder_id = ar_id
					cylinder_pose = i.pose.pose
					break
			try:
				if cylinder_id == ar_id and cylinder_pose.position.x > 0.1:
					break
			except:
				time.sleep(1)
		cylinder_poses.append([cylinder_id-marker_ids[0],cylinder_pose])
		print("...Cylinder ", ar_id, " Pose Extracted...")
	# launch.shutdown()
	if (len(cylinder_poses) < 1):
		return cylinder_pose_extractor()
	return cylinder_poses

if __name__ == '__main__':
	rospy.init_node("generate_env",anonymous=True)
	#Extract Robot Joint States
	table_pose = table_pose_extractor()
	
	# Get Cylinder Info
	# markers_ids = [4,5,6,7,8,9,10,11,12,13]
	markers_ids = [4,5,6,7,8]
	cylinder_poses = cylinder_pose_extractor(markers_ids)

	pose_dump = open(Config.PROJ_DIR+"real_world_exec/pose_dump.pickle","wb")
	pickle.dump([table_pose ,cylinder_poses], pose_dump)
	pose_dump.close()
	robot_joint_state_extractor()
	# import generate_cylinder_world
