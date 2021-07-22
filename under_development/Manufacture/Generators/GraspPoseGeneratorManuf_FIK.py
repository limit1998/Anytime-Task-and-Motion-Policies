import numpy as np
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
import openravepy
import math
import time


class GraspPoseGeneratorManuf_FIK(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["obj"]

        super(GraspPoseGeneratorManuf_FIK, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values["obj"]
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        self.number_of_values = 4

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

    def generate_function_err_free(self):
        for gt in self._compute_pose_list(False):
            yield gt
        raise StopIteration

    def generate_function_err(self):
        for gt in self._compute_pose_list(True):
            yield gt
        raise StopIteration

    def get_next(self, flag):
        if not flag["flag"]:
            return self.generate_function_state_err.next()
        else:
            return self.generate_function_state_err_free.next()

    def _compute_pose_list(self, remove_bodies):

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
        z_rotation_matrix[1][3] = body_y_len / 2 + .02
        # machine_T_robot[2][3] = -0.1

        world_T_robot = np.matmul(world_T_machine, np.matmul(z_rotation_matrix, x_rotation_matrix))

        # fetch_body.SetTransform(world_T_robot)
        gp = world_T_robot
        h = openravepy.misc.DrawAxes(env, gp)
        
        has_ik = OpenraveUtils.has_ik_to(env, self.simulator.env.GetRobots()[0], gp)
        print "has_ik: ", has_ik

        # import IPython
        # IPython.embed()
        time.sleep(1)
        h.Close()

        if has_ik:  # OpenraveUtils.has_ik_to(env, self.simulator.env.GetRobots()[0], gp):
            print "has IK to Grasp Pose\n"
            return [gp]
        else:
            return []
