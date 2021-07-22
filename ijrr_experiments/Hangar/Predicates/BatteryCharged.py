from src.DataStructures.Predicate import Predicate
import copy

class BatteryCharged(Predicate):
    def __init__(self,name,arg_list = None):
        super(BatteryCharged,self).__init__(name,arg_list)

    def __deepcopy__(self, memodict={}):
        return BatteryCharged(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self,ll_state,generated_values):
        ll_state.ll_variables["battery"] = 1000
