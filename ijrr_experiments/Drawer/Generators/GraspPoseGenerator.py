import copy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
from openravepy import *
import math


class GraspPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["can"]

        super(GraspPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.object_name = known_argument_values["can"]
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        self.number_of_values = 4


    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

    def generate_function_err_free(self):
        for gt in self._compute_pose_list(False):
            yield gt
        raise StopIteration

    def generate_function_err(self):
        for gt in self._compute_pose_list(True):
            yield gt
        raise StopIteration

    def get_next(self,flag):
        if not flag["flag"]:
            return self.generate_function_state_err.next()
        else:
            return self.generate_function_state_err_free.next()

    def _compute_pose_list(self,remove_bodies):
        object_name = self.object_name
        base_width, base_length, cylinder_height = self.simulator.get_object_link_dimensions(object_name=object_name, link_name='base')

        orig_transform = self.simulator.get_transform_matrix(object_name)





        # height_offset = -0.16
        if remove_bodies:
            name_to_object_and_transform = self.simulator.remove_all_removable_bodies(self.simulator.env)

        t2 = matrixFromPose([1,0,0,0,0,0,0.28])
        t3 = matrixFromAxisAngle([0,math.pi/2.0,0])
        t4 = np.eye(4)

        a = t4.dot(t2)
        b = a.dot(t3)

        final_t = orig_transform.dot(b)

        pose_list = []

        if len(self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(final_t,True)) > 0:
            pose_list.append(final_t)

        if remove_bodies:
            with self.simulator.env:
                for body_name in name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])

        # random.shuffle(pose_list)
        return pose_list




