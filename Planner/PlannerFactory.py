from OpenRavePlannerV2 import OpenRavePlannerV2
from TaskPlannerInterface.FastForwardTaskPlanner import FastForwardTaskPlanner
from FFPlanner import FFPlanner
from LAOSolver import LAOSolver

import Config

def create(planner_name):
    if planner_name == Config.OPENRAVE_PLANNER:
        return OpenRavePlannerV2()
    elif planner_name == Config.FF_PLANNER:
        return FFPlanner()
    elif planner_name == Config.LAO_SOLVER:
        return LAOSolver()

