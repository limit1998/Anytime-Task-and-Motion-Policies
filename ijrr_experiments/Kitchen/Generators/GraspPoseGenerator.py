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
        required_values = []

        super(GraspPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        # self.object_name = known_argument_values["item"]
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
        if "item" in self.known_argument_values:
            object_name = self.known_argument_values["item"]
        elif "tray" in self.known_argument_values:
            object_name = self.known_argument_values["tray"]

        env = self.simulator.env
        object = env.GetKinBody(object_name)

        object_transform = object.GetTransform()

        pose_list = []
        if "plate" in object_name:
            t1 = matrixFromPose([1,0,0,0,-0.26,0,0.05])
            r2 = matrixFromAxisAngle([0,0,-math.pi/2.0])
            r1 = matrixFromAxisAngle([math.pi/2.0,0,0])
            t3 = np.eye(4)
            pose =  t3.dot(r2).dot(t1).dot(r1)
            final_pose = object_transform.dot(pose)
            pose_list.append(final_pose)
        elif "cup" in object_name:
            t1 = matrixFromPose([1, 0, 0, 0, -0.17, 0, 0.1])
            r2 = matrixFromAxisAngle([0, 0, -math.pi/2.0])
            t3 = np.eye(4)
            pose = t3.dot(r2).dot(t1)
            final_pose = object_transform.dot(pose)
            pose_list.append(final_pose)
            pass
        elif "t" in object_name:
            offsets = [-0.2,0.2]
            t1 = matrixFromPose([1, 0, 0, 0, 0, 0.28, 0.07])
            r1 = matrixFromAxisAngle([0,0,-math.pi/2.0])
            t2 = matrixFromPose([1,0,0,0,0,-0.2,0])
            r2 = matrixFromAxisAngle([math.pi / 2.0, 0, 0])
            t3 = np.eye(4)
            pose = t3.dot(t1).dot(r1).dot(t2).dot(r2)
            final_pose = object_transform.dot(pose)
            pose_list.append(final_pose)
        return pose_list




