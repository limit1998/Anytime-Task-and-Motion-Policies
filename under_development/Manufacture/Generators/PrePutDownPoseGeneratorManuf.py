from src.DataStructures.Generator import Generator
import numpy as np
import Config
import json
import math
import openravepy
import src.OpenraveUtils as OpenraveUtils


class PutDownPoseGeneratorManuf(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['obj', 'loc']

        super(PutDownPoseGeneratorManuf, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_put_down_pose_list(self):
        generated_values_list = []
        env_config = json.load(open(Config.ENV_CONFIG_JSON, 'r'))
        env = self.simulator.env

        # object_name = self.known_argument_values['obj']
        target_location = self.known_argument_values['loc']

        machine_name = target_location[:-4]  # remove '_loc'
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
            # target_body = machine_base
            target_body = machine_name
            # target_body = "workstation{}".format(machine_num)

        body = env.GetKinBody(target_body)
        # body_pose = openravepy.poseFromMatrix(body.GetTransform())
        body_aabb = body.ComputeAABB()
        body_x_len = body_aabb.extents()[0] * 2
        body_y_len = body_aabb.extents()[1] * 2
        body_z_len = body_aabb.extents()[2] * 2
        body_center_x = body_aabb.pos()[0]
        body_center_y = body_aabb.pos()[1]
        # print "target:", body_center_x, body_center_y
        # print body_x_len, body_y_len, body_z_len

        if machine_type == "machine1" or machine_type == "machine2":
            z_rot_angle = 0
            x_rot_angle = np.pi / 2
            x_offset = body_x_len / 2 - .4
            y_offset = -body_y_len / 2
            z_offset = body_z_len + .1
        elif machine_type == "machine3":
            z_rot_angle = np.pi / 2
            x_rot_angle = np.pi / 2
            x_offset = 0
            y_offset = -body_y_len / 2 - .2
            z_offset = .2
        
        world_T_machine = body.GetTransform()

        # z_rot_angle = 0  # -np.pi/2 #0
        z_rotation_matrix = np.identity(4)
        z_rotation_matrix[0][0] = math.cos(z_rot_angle)
        z_rotation_matrix[0][1] = -(math.sin(z_rot_angle))
        z_rotation_matrix[1][0] = math.sin(z_rot_angle)
        z_rotation_matrix[1][1] = math.cos(z_rot_angle)

        # x_rot_angle = np.pi / 2
        x_rotation_matrix = np.identity(4)
        x_rotation_matrix[1][1] = math.cos(x_rot_angle)
        x_rotation_matrix[2][1] = math.sin(x_rot_angle)
        x_rotation_matrix[1][2] = -math.sin(x_rot_angle)
        x_rotation_matrix[2][2] = math.cos(x_rot_angle)

        # ply_object
        z_rotation_matrix[0][3] = x_offset  # body_x_len / 2 - .2
        z_rotation_matrix[1][3] = y_offset  # -body_y_len / 2
        z_rotation_matrix[2][3] = z_offset  # body_z_len + .1

        world_T_robot = np.matmul(world_T_machine, np.matmul(z_rotation_matrix, x_rotation_matrix))

        pose = openravepy.poseFromMatrix(world_T_robot)
        h = openravepy.misc.DrawAxes(env, pose)
        has_ik = OpenraveUtils.has_ik_to(env, self.simulator.env.GetRobots()[0], world_T_robot)
        print "has_ik: ", has_ik
        
        import IPython
        IPython.embed()

        h.Close()

        if has_ik:  # OpenraveUtils.has_ik_to(env, self.simulator.env.GetRobots()[0], world_T_robot):
            print "has IK to Grasp Pose"
            return [world_T_robot]

        return []


# z_rot_angle = np.pi / 2
# x_rot_angle = np.pi / 2
# x_offset = 0
# y_offset = -body_y_len / 2
# z_offset = .4

# z_rotation_matrix = np.identity(4)
# z_rotation_matrix[0][0] = math.cos(z_rot_angle)
# z_rotation_matrix[0][1] = -(math.sin(z_rot_angle))
# z_rotation_matrix[1][0] = math.sin(z_rot_angle)
# z_rotation_matrix[1][1] = math.cos(z_rot_angle)

# # x_rot_angle = np.pi / 2
# x_rotation_matrix = np.identity(4)
# x_rotation_matrix[1][1] = math.cos(x_rot_angle)
# x_rotation_matrix[2][1] = math.sin(x_rot_angle)
# x_rotation_matrix[1][2] = -math.sin(x_rot_angle)
# x_rotation_matrix[2][2] = math.cos(x_rot_angle)

# # ply_object
# z_rotation_matrix[0][3] = x_offset
# z_rotation_matrix[1][3] = y_offset
# z_rotation_matrix[2][3] = z_offset

# world_T_robot = np.matmul(world_T_machine, np.matmul(z_rotation_matrix, x_rotation_matrix))
# pose = openravepy.poseFromMatrix(world_T_robot)
# h = openravepy.misc.DrawAxes(env, pose)

# res = OpenraveUtils.has_ik_to(env, self.simulator.env.GetRobots()[0], world_T_robot)
# print(res)