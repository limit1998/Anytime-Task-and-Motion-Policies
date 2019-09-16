from TMP import TMP
import time
import pickle
from EnvGenerators import generate_cylinder_world
import Config
import subprocess
import os
import time
import multiprocessing

def run_cmd():
    TMP()

Config.SHOW_VIEWER = False
times = []
seeds = []
success = []
Config.RUN_TRAJ = False
Config.PLOT = True
Config.RESULTS_FILE = "results_uav_15.csv"
Config.DEFAULT_PDDL_FILE = Config.PROJ_DIR+'SampleTasks/hanger_15_domain.pddl'
Config.PORTNO = 1238

for ii in range(2):
    print "Starting Run No : {}".format(ii)
    generated_seed = int(time.time())
    print "Seed : ",generated_seed
    seeds.append(generated_seed)
    # generate_cylinder_world.create_env(Config.NUM_CANS,(0.03, 0.2),seed=generated_seed)
    killed = False
    start_time = time.time()
    try:
        p1 = multiprocessing.Process(target=run_cmd)
        p1.start()
    except Exception,e:
        ii -= 1
    else:
        while p1.is_alive():
            # print time.time()
            if time.time() - start_time > 600:
                p1.terminate()
                killed = True
        if killed:
            success.append(0)
        else:
            success.append(1)
            times.append(time.time() - start_time)
pickle.dump(times,open("uav_finalrun_times_"+str(Config.NUM_CANS)+"_15.p","wb"))
pickle.dump(seeds,open("uav_finalrun_seeds_"+str(Config.NUM_CANS)+"_15.p","wb"))
pickle.dump(success,open("uav_finalrun_success_"+str(Config.NUM_CANS)+"_15.p","wb"))




