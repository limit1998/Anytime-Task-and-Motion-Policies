import numpy as np
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
import time
import openravepy
import math


class PreGraspPoseGeneratorManuf_FIK(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["gpose", 'obj']

        super(PreGraspPoseGeneratorManuf_FIK, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.gpose = known_argument_values["gpose"]
        self.object_name = known_argument_values["obj"]
        self.offset = 0.10
        # self._compute_pose_list()
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        env = self.simulator.env

        model_name = self.object_name  # 'ply_object'
        body = env.GetKinBody(model_name)
        # body_pose = openravepy.poseFromMatrix(body.GetTransform())
        body_aabb = body.ComputeAABB()
        body_x_len = body_aabb.extents()[0] * 2
        body_y_len = body_aabb.extents()[1] * 2
        # body_z_len = body_aabb.extents()[2] * 2
        # body_center_x = body_aabb.pos()[0]
        # body_center_y = body_aabb.pos()[1]
        # print "target:", body_center_x, body_center_y
        # print body_x_len, body_y_len, body_z_len

        world_T_machine = body.GetTransform()

        # ply object
        z_rot_angle = -np.pi / 2
        z_rotation_matrix = np.identity(4)
        z_rotation_matrix[0][0] = math.cos(z_rot_angle)
        z_rotation_matrix[0][1] = -(math.sin(z_rot_angle))
        z_rotation_matrix[1][0] = math.sin(z_rot_angle)
        z_rotation_matrix[1][1] = math.cos(z_rot_angle)

        x_rot_angle = np.pi / 2
        x_rotation_matrix = np.identity(4)
        x_rotation_matrix[1][1] = math.cos(x_rot_angle)
        x_rotation_matrix[2][1] = math.sin(x_rot_angle)
        x_rotation_matrix[1][2] = -math.sin(x_rot_angle)
        x_rotation_matrix[2][2] = math.cos(x_rot_angle)

        # ply_object
        z_rotation_matrix[0][3] = body_x_len / 2
        z_rotation_matrix[1][3] = body_y_len / 2 + .1
        # machine_T_robot[2][3] = -0.1

        world_T_robot = np.matmul(world_T_machine, np.matmul(z_rotation_matrix, x_rotation_matrix))

        # fetch_body.SetTransform(world_T_robot)
        pgp = world_T_robot
        h = openravepy.misc.DrawAxes(env, pgp)
        has_ik = OpenraveUtils.has_ik_to(self.simulator.env, self.simulator.env.GetRobots()[0], pgp)
        print "has_IK:", has_ik

        # import IPython
        # IPython.embed()
        time.sleep(1)

        h.Close()

        print "Pre Grasp Pose", pgp

        if has_ik:
            print "has IK to Pre Grasp Pose\n"
            return [pgp]
        else:
            return []
