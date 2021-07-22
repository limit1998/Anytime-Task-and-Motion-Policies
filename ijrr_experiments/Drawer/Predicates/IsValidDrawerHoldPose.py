from src.DataStructures.Predicate import Predicate
import copy
from openravepy import *


class IsValidDrawerHoldPose(Predicate):
    def __init__(self, name, arg_list=None):
        super(IsValidDrawerHoldPose, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return IsValidDrawerHoldPose(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self, **kwargs):
        ll_state = kwargs['low_level_state']
        arg_map = kwargs["arg_map"]
        flag = kwargs["flag"]

        gp = arg_map["gppose"]

        simulator = ll_state.simulator
        robot = simulator.env.GetRobot(arg_map["robot"])

        if len(simulator.robots[arg_map["robot"]].get_ik_solutions(gp,True)) > 0:
            return True,[]
        else:
            iks = simulator.robots[arg_map["robot"]].get_ik_solutions(gp,False)
            ik = iks[0]
            old_dof = robot.GetActiveDOFValues()
            robot.SetActiveDOFValues(ik)
            co = []
            for body in simulator.env.GetBodies():
                if body.GetName() != arg_map["robot"]:
                    if simulator.env.CheckCollision(robot, body):
                        co.append(body.GetName())
            s = []
            for name in co:
                s.append("(obstructs_dd {} {})".format(name,arg_map["drawer"]))
            return False, []


    def get_failure_strings(self):
        pass