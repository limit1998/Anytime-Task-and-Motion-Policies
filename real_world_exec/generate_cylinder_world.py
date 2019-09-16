#! /usr/bin/python

import sys
import os
sys.path.append("/".join(os.path.abspath(os.path.abspath(__file__)).split('/')[:-2]))
import Config
sys.path.append(Config.PROJ_DIR +"/EnvGenerators")
from Robots.Models import FetchOpenRaveRobotModel

import random
import openravepy
import numpy as np
import utils
import settings
import time
import getopt
import object_models
import pickle
from tf.transformations import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

TARGET_OBJECT = "object1"
OBJECT_PREFIX = "object"
TABLE_NAMES = {"rll":"rll_table", "small":"table6"}

def execute(num_objects, cylinder_dims=(0.035, 0.33), max_radial_distance=0.73, robot_dist_from_table=0.15, seed=int(time.time()), envFile=settings.envFile, tabletype='rll', viewer=False):
    # Spawns PR2, table, and num_objects cylinders.
    # cylinder_dims: (radius, height)
    # max_radial_distance: x, y distance from PR2 which you want cylinders to be within
    # robot_dist_from_table: distance between front edge of PR2 and short edge of table (this is approximated)
    pose_dump = open("pose_dump.pickle","rb")
    table_pose, cylinder_poses = pickle.load(pose_dump)
    pose_dump.close()
    # assert max_radial_distance > robot_dist_from_table, "Robot too far from table"
    env = openravepy.Environment()
    if viewer:
        env.SetViewer('qtcoin')

    spawn_table_pr2(env, table_pose)
    # rand = np.random.RandomState(seed=seed)
    num_spawned = generate_world(env, cylinder_poses, 'table6', num_objects, cylinder_dims)
    on_table(env, 'table6')

    env.Save(envFile)
    # with open("seed.txt", 'w+') as f:
        # f.write(str(seed))
    print("Environment %s created; seed for %d objects attempted (%d actual): %d" %(envFile, num_objects, num_spawned, seed))
    return env

def generate_world(env, cylinder_poses, table_name, num_objects, cylinder_dims):
    collision_checker = openravepy.RaveCreateCollisionChecker(env, "fcl_")
    collision_checker.SetCollisionOptions(openravepy.CollisionOptions.Contacts)
    env.SetCollisionChecker(collision_checker)
    table = env.GetKinBody(table_name)
    count = 0
    for cylinder_id, cylinder_pose in cylinder_poses:
        print("ID:", cylinder_id)
    # for obj_num in range(num_objects):
        obj_name = "object" + repr(cylinder_id)
        if obj_name == TARGET_OBJECT:
            color = [0.2, 0.2, 0.2]
        else:
            color = [0, 0.8, 0.8]
        create_collision_free_random_cylinder(env, table, cylinder_dims, cylinder_pose, obj_name, color=color)
        body = env.GetKinBody(obj_name)
        if body is not None:
            print("Object %s created on surface %s" % (body.GetName(), table_name))
            count += 1
        else:
            print("Could not generate collision free cylinder for %s" % obj_name)
            
    return count

def create_collision_free_random_cylinder(env, table, dimensions, cylinder_pose, obj_name, color, num_trials=50):
    radius, height = dimensions

    for i in xrange(num_trials):
        cylinder = create_cylinder(env, table, radius, height, cylinder_pose, obj_name, color)
        collision = False
        for body in env.GetBodies():
            if body != table  and env.CheckCollision(body, cylinder):
                collision = True
                break
        if not collision:
            return
        env.Remove(cylinder)

def create_cylinder(env, table, radius, height, cylinder_pose, body_name, color):
    DIFF = 0.02
    min_x, max_x, min_y, max_y, table_height = utils.get_object_limits(table)

    # x = rand.uniform(min_x+DIFF + radius, max_x-DIFF - radius)
    # y = rand.uniform(min_y+DIFF + radius, max_y-DIFF - radius)
    x = cylinder_pose.position.x
    y = cylinder_pose.position.y
    # while (x - robot_pos[0])**2 + (y - robot_pos[1])**2 > max_radial_distance:
    #     x = rand.uniform(min_x + radius, max_x - radius)
    #     y = rand.uniform(min_y + radius, max_y - radius)
    z = table_height + height / 2

    t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
    cylinder = object_models.create_cylinder(env, body_name, t, [radius, height], color)
    env.Add(cylinder, False)
    
    return cylinder

# def spawn_table_pr2(env, table_pose,table_dims=(0.55,0.90,0.45)):
def spawn_table_pr2(env, table_pose,table_dims=(0.56,0.91,0.46)):
    length, breadth, height = table_dims
    x = table_pose.position.x
    y = table_pose.position.y
    # z = height/2
    # z = table_pose.position.z /2
    z = height/4
    # e_rot = euler_from_quaternion(numpy.asarray((table_pose.orientation.x,table_pose.orientation.y,table_pose.orientation.z,table_pose.orientation.w)))
    # print("e_rot", e_rot)

    # t = openravepy.matrixFromPose([table_pose.orientation.w, table_pose.orientation.x,table_pose.orientation.y,table_pose.orientation.z, x, y, z])
    t = openravepy.matrixFromPose([1,0,0,0, x, y, z])
    # print("table dims",table_dims )
    # print("table trans detected", table_pose)
    # print("table trans corrected", x,y,z)
    color = [0.5, 1.0, 0.5]
    table = object_models.create_box(env, "table6", t, [length/2, breadth/2 , height/2 ], color) #original
    env.Add(table, False)

def on_table(env, table_name):
    table = env.GetKinBody(table_name)
    if table:
      print("Transforming objects onto table")
      for obj in env.GetBodies():
          if obj.GetName().startswith(OBJECT_PREFIX):
              object_models.on_table(obj, table)


def create_env(num_objects, cylinder_dims, seed=int(time.time()), envFile=None,tabletype='small', viewer=False):
    if envFile is None:
        # dir = Config.PROJ_DIR+'GeneratedEnvironments/'
        envFile = Config.PROJ_DIR+'GeneratedEnvironments/real_can_world_'+str(num_objects)+"_cans.dae"
    execute(num_objects, cylinder_dims, seed=seed, envFile=envFile,tabletype=tabletype, viewer=viewer)
    if viewer:
        raw_input("press enter to quit")
        pass

if __name__ == "__main__":
    create_env(10,(0.032, 0.32),viewer=True)

    #seed=1548030364

