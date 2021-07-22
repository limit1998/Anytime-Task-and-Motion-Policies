from src.DataStructures.Generator import Generator
import numpy as np
from openravepy import poseFromMatrix, misc
import math
import json
import Config
from tf.transformations import euler_from_matrix
import random
import time


class BasePoseAtTargetGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['cbpose']
        super(BasePoseAtTargetGenerator, self).__init__(known_argument_values, required_values)
        self.known_argument_values = known_argument_values
        self.simulator = ll_state.simulator
        self.generate_function_state = self.generate_function()
        self.number_of_values = 4

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        transforms_list = self.generate_poses()
        # current_transform = self.simulator.env.GetRobot('fetch').GetTransform()
        # transforms_list.insert(0,[current_transform])
        for t in transforms_list:
            yield t

    def get_next(self, flag):
        return self.generate_function_state.next()

    def generate_poses(self):
        env = self.simulator.env
        env_config = json.load(open(Config.ENV_CONFIG_JSON, 'r'))

        target_location = self.known_argument_values['loc']
        print "target_location: ", target_location
        
        # The target location depends on the type of machine the robot has to go to
        if target_location == "object_init_location":
            target_body = "tool_shelf"
            machine_type = "tool_shelf"
        else:
            machine_name = target_location[:-4]  # removing "_loc" from "machine_1_loc"
            # rot_angle = env_config['machines'][machine_name]["rot_angle"]
            # loc_x = env_config['machines'][machine_name]["x"]
            # loc_y = env_config['machines'][machine_name]["y"]
            machine_base = env_config['machines'][machine_name]["machine_base_name"]
            workstation = env_config['machines'][machine_name]["workstation_name"]
            machine_type = env_config['machines'][machine_name]["type"]

            target_body = machine_name
            if machine_type == "machine1" or machine_type == "machine2":
                target_body = workstation
                # target_body = "{}_base".format(machine_name)
            elif machine_type == "machine3":
                # machine_num = machine_name[7:]
                target_body = machine_base
                # target_body = machine_name
                # target_body = "workstation{}".format(machine_num)
            elif machine_type == "machine4":
                target_body = machine_name

        print "target body: ", target_body
        body = env.GetKinBody(target_body)
        # body_pose = poseFromMatrix(body.GetTransform())
        body_aabb = body.ComputeAABB()
        body_x_len = body_aabb.extents()[0] * 2
        body_y_len = body_aabb.extents()[1] * 2
        # body_z_len = body_aabb.extents()[2] * 2
        # body_center_x = body_aabb.pos()[0]
        # body_center_y = body_aabb.pos()[1]
        
        body_length = body_x_len if body_x_len > body_y_len else body_y_len  # longer side
        body_breadth = body_y_len if body_x_len > body_y_len else body_x_len  # shorter side

        base_pose_list = []
        pose_axes = []
        for i in range(0, Config.BASE_POSE_COUNTS):
            if machine_type == "machine1" or machine_type == "machine2":
                # rotate robot to face target body
                rot_angle = 0
                # offset x distance from target body
                x_offset = random.uniform(-.6, -.5)
                # offset y distance from target body
                y_offset = random.uniform(.1, -body_length + .1)
            
            elif machine_type == "machine3":
                rot_angle = 0
                x_offset = random.uniform(-.7, -.5)
                y_offset = random.uniform(.1, -body_length + .1)
                
            elif machine_type == "machine4":
                rot_angle = np.pi / 2
                x_offset = random.uniform(0, body_breadth)
                y_offset = -body_length + random.uniform(-.7, -.5)
                
            elif machine_type == "tool_shelf":
                rot_angle = 0
                x_offset = random.uniform(-1.5 * body_breadth, -body_breadth)
                y_offset = random.uniform(0, -body_length)

            T_machine_wrt_world = body.GetTransform()

            T_robot_wrt_machine = np.identity(4)
            T_robot_wrt_machine[0][0] = math.cos(rot_angle)
            T_robot_wrt_machine[0][1] = -(math.sin(rot_angle))
            T_robot_wrt_machine[1][0] = math.sin(rot_angle)
            T_robot_wrt_machine[1][1] = math.cos(rot_angle)

            T_robot_wrt_machine[0][3] = x_offset
            T_robot_wrt_machine[1][3] = y_offset
            # print "rot_angle: ", rot_angle
            # print "x_offset: ", x_offset
            # print "y_offset: ", y_offset

            world_T_robot = np.matmul(T_machine_wrt_world, T_robot_wrt_machine)
            target_pose = poseFromMatrix(world_T_robot)
            h = misc.DrawAxes(env, target_pose)
            pose_axes.append(h)

            euler = euler_from_matrix(world_T_robot)

            theta = euler[2]
            if theta == -np.pi / 2:
                theta = 3 * np.pi / 2
            x = target_pose[4]
            y = target_pose[5]

            print "BASE TARGET POSE:", x, y, theta

            # h.Close()

            base_pose_list.append([x, y, theta])
        
        time.sleep(1)
        for h in pose_axes:
            h.Close()
        return base_pose_list
