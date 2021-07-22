from src.DataStructures.Generator import Generator
import numpy as np
from openravepy import *
import math


class BasePoseAroundTableGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['cbpose']
        super(BasePoseAroundTableGenerator, self).__init__(known_argument_values, required_values)
        self.known_argument_values = known_argument_values
        self.simulator = ll_state.simulator
        self.generate_function_state = self.generate_function()
        self.number_of_values = 4

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        transforms_list = self.generate_poses()
        # current_transform = self.simulator.env.GetRobot('fetch').GetTransform()
        # transforms_list.insert(0,[current_transform])
        for t in transforms_list:
            yield t

    def get_next(self,flag):
        return self.generate_function_state.next()


    def generate_poses(self):
        env = self.simulator.env

        transform_list = []

        object_name = None
        if "tray" in self.known_argument_values:
            object_name = self.known_argument_values["tray"]
        elif "item" in self.known_argument_values:
            object_name = self.known_argument_values["item"]

        location = self.known_argument_values["location"]

        relative_pose = [1, 0, 0, 0, 0, 0.75, -0.5]
        table_relative_pose = [1, 0, 0, 0, 0, 0.95, -0.5]
        r = matrixFromAxisAngle([0, 0, -math.pi / 2.0])


        if object_name is None:
            table_transform = env.GetKinBody(location).GetTransform()

            new_pose = table_transform.dot(matrixFromPose(table_relative_pose)).dot(r)
        else:

            object_transform = env.GetKinBody(object_name).GetTransform()
            if "tray" in self.known_argument_values:
                new_pose = object_transform.dot(matrixFromPose(table_relative_pose)).dot(r)
            else:
                new_pose = object_transform.dot(matrixFromPose(relative_pose)).dot(r)


        final_x = new_pose[0,3]
        final_y = new_pose[1,3]
        final_theta = axisAngleFromRotationMatrix(new_pose)[2]

        transform_list.append([final_x,final_y,final_theta])

        return transform_list

