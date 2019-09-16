DOMAIN_PATH = "../domains/"
errFileName = "robotics_autogen_err1.txt"
ENVPATH="../environments/"

CAN_DOMAIN = 0
DINNER_DOMAIN = 1
DRAWER_DOMAIN = 2
LAUNDRY_DOMAIN = 3
KITCHEN_DOMAIN = 4
RESTAURANT_DOMAIN = 5
# DOMAIN = CAN_DOMAIN

use_ros = False
def toggle_use_ros():
    global use_ros
    use_ros = True

def init_settings():
    global TRAJOPT_DEBUG
    TRAJOPT_DEBUG = False

    import time
    import numpy as np
    global seed
    seed = int(time.time())
    seed = 123456
    print "SEED: {}".format(seed)

    global RANDOM_STATE
    RANDOM_STATE = np.random.RandomState(seed)

FF = "ff"
MP = "mp"
FD = "fd"
FDOPTIMALMODE = True
FFEXEC = "../planners/FF-v2.3/ff"
FDEXEC = "../planners/FD/src/plan-ipc seq-sat-fd-autotune-1 "
FDOPTEXEC = "../planners/FD/src/plan-ipc seq-opt-lmcut "
MPEXEC = "../planners/M/Mp"

def set_domain(dom):
    global DOMAIN, pddlDomainFile, pddlDomainFileNoGeomEff, initialProblemFile, PLANNER_TO_USE, REPORT_PUTDOWN_OBSTRUCTIONS
    DOMAIN = dom
    if dom == CAN_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc.pddl"
        pddlDomainFileNoGeomEff = DOMAIN_PATH+ "robotics_twoarms_objloc_ignore_obs.pddl"
        initialProblemFile = DOMAIN_PATH + "robotics_twoarms_50cans_prob_objloc.pddl"
        PLANNER_TO_USE = FF
    elif dom == DINNER_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "dinnerTime_handoff_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_prob_4obj.pddl"
        PLANNER_TO_USE = FD
    elif dom == DRAWER_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "drawer_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "drawer_prob.pddl"
        PLANNER_TO_USE = FF
    elif dom == LAUNDRY_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "laundry_onemachine_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "laundry_onemachine_prob.pddl"
        PLANNER_TO_USE = FD
    elif dom == KITCHEN_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "kitchen_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "kitchen_prob_test.pddl"
        PLANNER_TO_USE = FD
    elif dom == RESTAURANT_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "restaurant_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "restaurant_prob.pddl"
        PLANNER_TO_USE = FD

    if 'surfaceloc' in pddlDomainFile or 'putdownloc' in pddlDomainFile:
        REPORT_PUTDOWN_OBSTRUCTIONS = True
    else:
        REPORT_PUTDOWN_OBSTRUCTIONS = False

# pddlDomainFile = DOMAIN_PATH #+ raw_input("Enter PDDL domain file name: ")
# if pddlDomainFile == DOMAIN_PATH:
    ## not used
    #pddlDomainFile = DOMAIN_PATH+ "robotics_obstrn_compiled_dom2.pddl"
    #pddlDomainFile = DOMAIN_PATH+ "robotics_obstrn_compiled_dom2_typed_twoarms.pddl"

    # pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc_surfaceloc.pddl"
    # pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc_putdownloc_lowmem_dom.pddl"
    # pddlDomainFile = DOMAIN_PATH + "robotics_twoarms_objloc.pddl"
    # pddlDomainFileNoGeomEff = DOMAIN_PATH+ "robotics_twoarms_objloc_ignore_obs.pddl"


     # for ff, dinnertime:
     # pddlDomainFile = DOMAIN_PATH+ "dinnerTime_dom.pddl"
     #pddlDomainFile = DOMAIN_PATH+ "dinnerTimeNoNegation_dom.pddl"
    #pddlDomainFile = DOMAIN_PATH+ "dinnerTimeNoNegationCosts_dom.pddl"
    #pddlDomainFile = DOMAIN_PATH + "dinnerTime_twoarms_dom_costs.pddl"
    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_onearm_costs_fast_dom.pddl"
    #pddlDomainFile = DOMAIN_PATH + "dinnerTime_twoarms_compiledcosts_fast_dom.pddl"
    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_twoarms_costs_fast_dom.pddl"
    # pddlDomainFile = DOMAIN_PATH+ "dinnerTimeCompiledCosts_dom.pddl"

    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_handoff_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile

    # pddlDomainFile = DOMAIN_PATH + "dinnerTime_handoff_fixedheight_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile

    # pddlDomainFile = DOMAIN_PATH + "drawer_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile

    # pddlDomainFile = DOMAIN_PATH + "laundry_onemachine_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile

    # pddlDomainFile = DOMAIN_PATH + "dishwasher_dom.pddl"
    # pddlDomainFileNoGeomEff = pddlDomainFile

# initialProblemFile = DOMAIN_PATH #+ raw_input("Enter PDDL problem file name: ")
# if initialProblemFile == DOMAIN_PATH:
    #initialProblemFile = DOMAIN_PATH + "robotics_autogen_prob_65objs_typed.pddl"
    #initialProblemFile = DOMAIN_PATH + "canworld_gazebo_12cans_twoarms.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_12cans_prob_objloc.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_22cans_prob_objloc.pddl"
    #initialProblemFile = DOMAIN_PATH + "robotics_twoarms_22cans_objloc_fixedbase_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_50cans_prob_objloc.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_50cans_prob_objloc_surfaceloc.pddl"
    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_objloc_putdownloc_lowmem_50cans_prob.pddl"

    # initialProblemFile = DOMAIN_PATH + "robotics_twoarms_22cans_objloc_fixedbase_prob.pddl"

    #for ff, dinnertime:
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_prob.pddl"
    #initialProblemFile = DOMAIN_PATH + "dinnerTimeNoNegation_prob.pddl"
    #initialProblemFile = DOMAIN_PATH + "dinnerTimeNoNegationCostsTargets_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_onearm_costs_fast_prob.pddl"
    #initialProblemFile = DOMAIN_PATH + "dinnerTime_twoarms_compiledcosts_fast_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_twoarms_costs_fast_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTimeCompiledCosts_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_prob_4obj.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_prob_2obj.pddl"
    # initialProblemFile = DOMAIN_PATH + "dinnerTime_handoff_fixedheight_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "drawer_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "laundry_onemachine_prob.pddl"
    # initialProblemFile = DOMAIN_PATH + "dishwasher_prob_test.pddl"



collision_free_grasping_samples = 1
occluding_objects_grasping_samples = 1

doJointInterpretation = False
OpenRavePlanning = False

envFile = ENVPATH+"created_info.dae"
# envFile = ENVPATH + "rll_tray_world.dae"
#envFile = ENVPATH + "rll_dinner_demo_handoff.dae"

DISABLE_BASE = False
DO_BACKTRACKING = True
REPLAN = False
REDETECT = False
USE_SOFT_LIMITS = False
PRINT_GEN_COUNT = False
USE_MAX_ITERATIONS = False
APPLY_DSH_PATCH = False
INCREASE_MARGINS = False

# runs hybridPlanner automatically, no raw_inputs, use seed that generation script used, and only motion plan, no actual execution
# second arg is seconds before planning times out
run_test_mode = (False, 65000)

if run_test_mode[0]:
    try:
        with open("../tests/seed.txt", 'r') as f:
            seed = int(f.read())
    except (IOError, ValueError):
        print("Could not read seed that generation script used!")

