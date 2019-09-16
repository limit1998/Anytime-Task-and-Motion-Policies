from TMP import TMP
import time
import pickle
from EnvGenerators import generate_cylinder_world
import Config


Config.SHOW_VIEWER = False
times = []
seeds = []

try:
    for ii in range(5):
        print ii
        generated_seed = int(time.time())
        print "Seed : ",generated_seed
        seeds.append(generated_seed)
        generate_cylinder_world.create_env(10,(0.03, 0.2),seed=generated_seed)
        start_time = time.time()
        TMP();
        time_taken = time.time() - start_time
        times.append(time_taken)
except Exception,e:
    pickle.dump(times, open("10_cans_times.p", "wb"))
    pickle.dump(seeds, open("10_cans_seeds.p", "wb"))
else:
    pickle.dump(times,open("10_cans_times.p","wb"))
    pickle.dump(seeds,open("10_cans_seeds.p","wb"))