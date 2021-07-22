from src.DataStructures.Predicate import Predicate
import copy
from openravepy import *

class BatteryConsumed(Predicate):
    def __init__(self,name,arg_list = None):
        super(BatteryConsumed,self).__init__(name,arg_list)

    def __deepcopy__(self, memodict={}):
        return BatteryConsumed(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self,ll_state,generated_values):
        traj = generated_values["trajectory"]
        trajobj = RaveCreateTrajectory(ll_state.simulator.env, '')
        Trajectory.deserialize(trajobj, traj)
        l = trajobj.GetNumWaypoints()
        ll_state.ll_variables["battery"] -= (l /2.0)