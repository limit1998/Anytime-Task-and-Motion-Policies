import pickle
import subprocess
import os
import time
import multiprocessing
import random
import TMP


def run_cmd():
    TMP.TMP().execute()

times = []
seeds = []
success = []


TMP.Config.NUM_CANS = '15'

for ii in range(30):
    generated_seed = int(time.time())
    print "seed: ", generated_seed
    random.seed(generated_seed)
    os.chdir("./ijrr_experiments/DelicateCanDeterministic/Environments/")
    os.system("python generate_cylinder_world.py {} {}".format(15,generated_seed))
    os.chdir("../../../")
    time.sleep(1)
    print "Starting run no {}".format(ii)
    killed = False
    start_time = time.time()
    try:
        p1 = multiprocessing.Process(target=run_cmd)
        p1.start()
    except Exception,e:
        ii -= 1
    else:
        while p1.is_alive():
            if time.time() - start_time > 4000:
                p1.terminate()
                killed = True
        if killed:
            success.append(0)
        else:
            success.append(1)
        times.append(time.time() - start_time)
    pickle.dump(times,open("ijrr_experiments/DelicateCanDeterministic/results/delicate_cans_15_times.p", "wb"))
    pickle.dump(seeds,open("ijrr_experiments/DelicateCanDeterministic/results/delicate_cans_15_seeds.p", "wb"))
    pickle.dump(success,open("ijrr_experiments/DelicateCanDeterministic/results/delicate_cans_15_success.p","wb"))

# f = open("./ijrr_experiments/DelicateCanDeterministic/DomainConfig.py","r")
# d = f.read()
# new_d = d.replace('NUM_CANS = "20"', 'NUM_CANS = "25"')
# with open("./ijrr_experiments/DelicateCanDeterministic/DomainConfig.py","w") as f:
#     f.write(new_d)
#



