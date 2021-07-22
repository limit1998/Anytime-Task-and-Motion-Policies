import sys,os

DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'


# Manufacturing ENV
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/manuf_world_domain.pddl'  # PATH to the PPDDL/PDDL domain file
DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/manuf_world_problem.pddl'  #  PATH to the PDDL/PPDDL problem file
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/manuf_world.output'  # Optional : PATH to the solution file

#unzipping env files
if not os.path.isfile(DOMAIN_DIR+'Environments'):
    import tarfile
    my_tar = tarfile.open(DOMAIN_DIR+'Environments.tar.gz')
    my_tar.extractall(DOMAIN_DIR)
    my_tar.close()

# OPENRAVE_ENV_XML = DOMAIN_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae' # Low level envrionment dae/xml path
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/env.dae'
ENV_CONFIG_JSON = DOMAIN_DIR+'Environments/env_config.json'
# OPENRAVE_ENV_XML = PROJ_DIR + "GeneratedEnvironments/real_can_world_10_cans.dae"
ROBOT_NAME = 'fetch'  # NAME of the robot. Should match the robot name in the environment
DOMAIN_NAME = "manuf_world_small"  # name of the domain. Optional
LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_manufworld.json'  # PATH to actionConfig file
REAL_ROBOT = False  
BASE_POSE_COUNTS = 7
IK_SOLVER = "ik_fast"
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
HL_PLANNER = FF_PLANNER