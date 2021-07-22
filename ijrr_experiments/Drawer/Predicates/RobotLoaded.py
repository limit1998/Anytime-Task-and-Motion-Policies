from src.DataStructures.Predicate import Predicate
import copy
import numpy as np
from openravepy import *

class RobotLoaded(Predicate):
    def __init__(self, name, arg_list=None):
        super(RobotLoaded, self).__init__(name, arg_list)
        self.arg_list = arg_list

    def __deepcopy__(self, memodict={}):
        return RobotLoaded(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        robot = ll_state.simulator.env.GetRobot("fetch")
        t = matrixFromPose([0.70710678,  0, 0, -0.70710678,  0.3, 0.55,  0.])
        # t = matrixFromPose([0.70710678,  0, 0, -0.70710678,  0.3, 0.55,  0.])
        robot.SetTransform(t)