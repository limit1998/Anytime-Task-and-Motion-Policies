from src.DataStructures.Predicate import Predicate
import copy
from openravepy import *

class IsBatterySufficient(Predicate):
    def __init__(self, name, arg_list=None):
        super(IsBatterySufficient, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return IsBatterySufficient(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __call__(self,**kwargs):
        loc = kwargs["arg_map"]
        if loc == "recharge_station":
            return True,[]
        else:
            traj = kwargs["arg_map"]["trajectory"]
            robot_name = kwargs["arg_map"]["agent"]
            htraj = kwargs["arg_map"]["htraj"]
            ll_state = kwargs['low_level_state']
            trajobj = RaveCreateTrajectory(ll_state.simulator.env, '')
            Trajectory.deserialize(trajobj, traj)
            l = trajobj.GetNumWaypoints()
            ll_state = kwargs['low_level_state']
            if ll_state.ll_variables["battery"] > (l/2.0):
                return True,[]
            else:
                return False, ["(not (hasBattery {] {}".format(robot_name,htraj)]
