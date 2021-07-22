#!/usr/bin/env python
import rospy
import openravepy as orpy
from gazebo_msgs.msg import ModelStates


class GazeboToORTranslator(object):

    def __main__(self):
        self.mesh_folder_dict = {"cup" : "cup_green", "plate" : "plate_2", "bottle" : "bottle_red_wine", "table" : "table", "marble" : "table_marble"}
        self.item_pose = None
        self.subscriber = rospy.Subscriber("/gazebo/model_states",ModelStates,self.pose_callback)

    def pose_callback(self,data):
        if self.item_pose is None:
            self.item_pose = data
            self.convert()
        else:
            pass

    def convert(self):

        or_env = orpy.Envrionment()
        for object_name in self.item_pose.name:
            for i,name in enumerate(self.mesh_folder_dict.keys()):
                if name in object_name:
                    or_obj = or_env.ReadKinBodyXMLFile("/home/naman/.gazebo/models/"+self.mesh_folder_dict[name]+"/meshes/model.dae")
                    pose = self.mesh_folder_dict.pose[i]
                    or_pose = (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.position.x, pose.position.y, pose.position.z) 
                    or_obj.SetName(object_name)
                    or_obj.SetTransformPose(or_pose)
        or_env.Save("kitchen_or_world.dae")

if __name__ == "__main__":
    try:
        GazeboToORTranslator()
    except Exception,e:
        print e
        exit(-1)


