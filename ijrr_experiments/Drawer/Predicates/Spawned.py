from src.DataStructures.Predicate import Predicate
import copy
import numpy as np

class Spawned(Predicate):
    def __init__(self, name, arg_list=None):
        super(Spawned, self).__init__(name, arg_list)
        self.arg_list = arg_list

    def __deepcopy__(self, memodict={}):
        return Spawned(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        next_hl_state = generated_values["next_hl_state"].getTrueProps()
        for prop in next_hl_state:
            if "(in " == prop[:4] and generated_values["can"] in prop:
                can1 = ll_state.simulator.env.GetKinBody("can1")
                drawer_name = prop.split(" ")[2][:-1]
                drawer = ll_state.simulator.env.GetKinBody(drawer_name).GetTransform()
                drawer[0,3] -= 0.2
                can1.SetTransform(drawer)