from src.DataStructures.Generator import  Generator
import Config
import math
import numpy as np
from openravepy import *


class DrawerHoldPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["drawer"]

        super(DrawerHoldPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.drawer_name = self.known_argument_values["drawer"]
        self.generator = self.get_generator()
        self.generator_err = self.get_generator_err()

    def reset(self):
        self.generator = self.get_generator()
        self.generator_err = self.get_generator_err()

    def get_generator_err(self):
        for pose in self._compute_list(True):
            yield pose
        raise StopIteration

    def get_generator(self):
        for pose in self._compute_list(False):
            yield pose
        raise StopIteration


    def _compute_list(self,remove_bodies):
        env = self.simulator.env
        drawer = env.GetKinBody(self.drawer_name)
        drawer_t = drawer.GetTransform()

        if remove_bodies:
            name_to_object_and_transform = self.simulator.remove_all_removable_bodies(self.simulator.env)


        rt = matrixFromAxisAngle([math.pi/2.0,0,0])
        drawer_aabb = drawer.ComputeAABB()
        x = drawer_aabb.extents()[0]
        t = np.eye(4)
        t[0,3] -= (x + 0.05 + 0.17)
        t[2,3] += 0.05

        t2 = t.dot(rt)
        final_t = drawer_t.dot(t2)

        poses = []

        if len(self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(final_t, True)) > 0:
            poses = [final_t]
        else:
            pass

        if remove_bodies:
            with self.simulator.env:
                for body_name in name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])

        return poses



    def get_next(self,flag):
        if not flag["flag"]:
            return self.generator_err.next()
        else:
            return self.generator.next()
