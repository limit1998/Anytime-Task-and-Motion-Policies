from openravepy import *
import object_models
import utils
import numpy as np
import sys
import time

if len(sys.argv) > 3:
    seed = int(sys.argv[2])
else:
    seed = int(time.time())

VIEWER = True

np.random.seed(seed)


def create_cylinder(env, table, radius, height, body_name, color):
    DIFF = 0.02
    min_x, max_x, min_y, max_y, table_height = utils.get_object_limits(table)

    x = np.random.uniform(min_x + DIFF + radius, max_x - DIFF - radius)
    y = np.random.uniform(min_y + DIFF + radius, max_y - DIFF - radius)
    # z = table_height + height / 2
    z = table_height + 0.001

    t = matrixFromPose([1, 0, 0, 0, x, y, z])
    cylinder = object_models.create_cylinder(env, body_name, t, [radius, height], color)
    env.Add(cylinder)
    return cylinder


def add_cylinder(env,table,cylinder_number,color = [1,0,0]):
    body_name = "can"+str(cylinder_number)
    while True:
        cylinder = create_cylinder(env,table,0.03,0.20,body_name,color)
        colision = False
        for body in env.GetBodies():
            if env.CheckCollision(body,cylinder) and cylinder.GetName() != body.GetName():
                print body.GetName()
                colision = True
                break

        if not colision:
            break
        else:
            env.Remove(cylinder)



if __name__ == "__main__":

    n = 10
    # n = int(sys.argv[1])

    env = Environment()
    objects_str = ""
    objects_color_str = ""
    objects_loc_str = ""
    env.Load("base_env.dae")
    collision_checker = RaveCreateCollisionChecker(env, "pqp")
    collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
    env.SetCollisionChecker(collision_checker)
    if VIEWER:
        env.SetViewer('qtcoin')
    table = env.GetKinBody("main_table")

    for i in range(1,n+1):
        add_cylinder(env,table,i,color=[0,1,0])
        objects_str += "can{}".format(i) + " "
        objects_loc_str += "(at_obj can{} main_table)".format(i) + "\n"
        objects_color_str += "(color can{} green)".format(i) + "\n"

    for i in range(n+1, 2*n+1):
        add_cylinder(env,table,i,color=[0,0,1])
        objects_str += "can{}".format(i) + " "
        objects_loc_str += "(at_obj can{} main_table)".format(i) + "\n"
        objects_color_str += "(color can{} blue)".format(i) + "\n"

    for i in range(2*n+1, 4*n + 1):
        add_cylinder(env,table,i,color=[1,0,0])
        objects_str += "can{}".format(i) + " "
        objects_loc_str += "(at_obj can{} main_table)".format(i) + "\n"
        objects_color_str += "(color can{} red)".format(i) + "\n"

    if VIEWER:
        raw_input("Press any key")

    txt = open("../Tasks/deterministic_problem_template.pddl","r").read()
    txt = txt.replace("##OBJECTS_HERE##",objects_str)
    txt = txt.replace("##OBJ_INIT_LOC_HERE##",objects_loc_str)
    txt = txt.replace("##COLORS_HERE##",objects_color_str)
    fout = open("../Tasks/deterministic_problem_{}.pddl".format(n),"w")
    fout.write(txt)
    fout.close()
    env.Save("sort_clutter_{}.dae".format(n))