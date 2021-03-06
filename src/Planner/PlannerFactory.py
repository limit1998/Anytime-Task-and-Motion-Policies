from OpenRavePlannerV2 import OpenRavePlannerV2
from src.Planner.FFPlanner import FFPlanner
from src.Planner.LAOSolver import LAOSolver

import Config

def create(planner_name):
    # if planner_name == Config.OPENRAVE_PLANNER:
    #     return OpenRavePlannerV2()
    if planner_name == Config.FF_PLANNER:
        return FFPlanner()
    elif planner_name == Config.LAO_SOLVER:
        return LAOSolver()

