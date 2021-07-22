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
        table_name = self.known_argument_values["table"]
        table = env.GetKinBody(table_name)
        table_pose = table.GetTransform()
        relative_poses = [[1,0,0,0,0,0.75,-0.5],
                          [1,0,0,0,-0.35,0.75,-0.5],
                          [1,0,0,0,0.35,0.75,-0.5]]
        r = matrixFromAxisAngle([0,0,-math.pi/2.0])

        base_poses = []
        for pose in relative_poses:
            final_pose = table_pose.dot(matrixFromPose(pose)).dot(r)
            final_x = final_pose[0,3]
            final_y = final_pose[1,3]
            final_theta = axisAngleFromRotationMatrix(final_pose)[2]
            base_poses.append([final_x,final_y,final_theta])
        return base_poses

