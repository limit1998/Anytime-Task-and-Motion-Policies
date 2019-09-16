from TMP import TMP
import time
import pickle
from EnvGenerators import generate_cylinder_world
import Config
import subprocess
import os
import time

def run_cmd():
    os.system("python TMP.py")

Config.SHOW_VIEWER = False
Config.NUM_CANS = 3
times = []
seeds = []
Config.RUN_TRAJ = False
Config.PLOT = True


try:
    for ii in range(20):
        print "Starting Run No : {}".format(ii)
        generated_seed = int(time.time())
        print "Seed : ",generated_seed
        seeds.append(generated_seed)
        generate_cylinder_world.create_env(Config.NUM_CANS,(0.03, 0.2),seed=generated_seed)
        start_time = time.time()
        p1 = subprocess.Popen("python TMP.py")
        while True:
            p1.poll()
        exit()
        time_taken = time.time() - start_time
        times.append(time_taken)
except Exception,e:
    print e
    pickle.dump(times,open("canworld_time_runs_02_24_times.p","wb"))
    pickle.dump(seeds,open("canworld_time_runs_02_24_seeds.p","wb"))
else:
    pickle.dump(times, open("canworld_time_runs_02_24_times.p", "wb"))
    pickle.dump(seeds, open("canworld_time_runs_02_24_seeds.p", "wb"))

