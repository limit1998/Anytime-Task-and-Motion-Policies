import copy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import time

class DrawerOpenPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["gppose"]

        super(DrawerOpenPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.gpose = known_argument_values["gppose"]
        self.offset = 0.30 # user defined pregrasp offset
        self.robot_name = self.known_argument_values["robot"]
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        env = self.simulator.env
        t0 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
        if self.known_argument_values["drawer"] == "bottom_drawer":
            self.offset = 0.40
        t1 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, -self.offset, 0, 0)
        tpgp_wrt_gp = t0.dot(t1)
        pgp = self.gpose.dot(t1)
        if len(self.simulator.robots[self.robot_name].get_ik_solutions(pgp,True))>0:
            return [pgp]
        else:
            return []