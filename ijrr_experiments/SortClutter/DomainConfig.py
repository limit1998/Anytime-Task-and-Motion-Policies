import os
### FOR Delicate Canworld MDP
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'
  # True if need to match init position with real robot, true if executing final results on real robot.

NUM_CANS = "3"
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/deterministic_domain.pddl'  # PATH to the PPDDL/PDDL domain file
DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/deterministic_problem_'+NUM_CANS+'.pddl'  #  PATH to the PDDL/PPDDL problem file
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/deterministic_problem_'+NUM_CANS+'_soln.output'  # Optional : PATH to the solution file
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/sort_clutter_'+NUM_CANS+'.dae' # Low level envrionment dae/xml path
# OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/temp.dae' # Low level envrionment dae/xml path
ROBOT_NAME = 'fetch'  # NAME of the robot. Should match the robot name in the environment
DOMAIN_NAME = "sort_clutter"  # name of the domain. Optional
# LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_delicate_cans_V3.json'  # PATH to actionConfig file
LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig.json'  # PATH to actionConfig file with prpy support
REAL_ROBOT = False  # True if need to match init position with real robot, true if executing final results on real robot.
IK_SOLVER = "ik_fast"
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
HL_PLANNER = FF_PLANNER
LOOPED_RUNS = False
BACKTRACK_PROB = 0.0
MAX_TIME = 1000000
RESULTS_FILE = "result_sort_{}.csv".format(NUM_CANS)