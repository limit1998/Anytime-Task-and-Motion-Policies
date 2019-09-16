from openravepy import *
import numpy as np
import random
import OpenraveUtils
from Generator import Generator
from Simulators.OpenRaveSimulator import *
import math
import random
from trac_ik_python.trac_ik import IK
import Config

class GraspPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'plank1', 'region']

        super(GraspPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('plank1')
        self.grasp_region = known_argument_values.get('region')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def set_IK_solver(self, base, tip, urdf_str):
        ik_solver = IK(base, tip, urdf_string=urdf_str)
        return ik_solver

    def get_IK_solution(self, ik_solver, seed_state, trans, quat):
        max_count = 10
        count = 0
        while True and count < max_count:
            sol = ik_solver.get_ik(seed_state,
                                   trans[0], trans[1], trans[2],  # X, Y, Z
                                   quat[1], quat[2], quat[3], quat[0]  # QX, QY, QZ, QW
                                   )
            if sol is None:
                print "No Solution...Retrying"
                count += 1
            else:
                print "IK Solution Found"
                return list(sol)

        return sol

    def set_DOFs_for_IK_solution(self, solution):
        with self.env:
            try:
                self.robot.SetActiveDOFValues(solution)
            except:
                pass

    def get_urdf_string(self):
        with open(Config.FETCH_URDF, 'r') as file:
            urdf_str = file.read()

        return urdf_str

    def _compute_pose_list(self):

        # ToDo: Transforms from robot base

        pose_list =[]
        grasp_num = 10
        region_length = 0.117475/3.0
        gripper_offset = 0.14 # 0.35 Initial value
        rot_angle = math.pi / 2.0
        _, region = self.grasp_region.split('_')
        offset_mean = (region_length * int(region)) - (2*region_length)

        grasp_offsets = []
        for i in range(grasp_num):
            grasp_offsets.append(random.uniform(offset_mean-region_length/2.0, offset_mean+region_length/2.0))

        self.env = self.simulator.env
        self.robot = self.env.GetRobots()[0]

        self.table = self.env.GetKinBody('table60')
        base_link = self.robot.GetLinks()[0]
        base_transform = base_link.GetTransform()
        object = self.env.GetKinBody(self.object_name)
        object_transform = object.GetTransform()

        # Change the YuMi URDF. World -> BaseLink
        robot_world_transform = self.robot.GetLink('world').GetTransform()

        roll_angle = math.pi
        roll_rot_matrix = np.identity(4)
        roll_rot_matrix[1][1] = math.cos(roll_angle)
        roll_rot_matrix[1][2] = -math.sin(roll_angle)
        roll_rot_matrix[2][1] = math.sin(roll_angle)
        roll_rot_matrix[2][2] = math.cos(roll_angle)

        yaw_angle = -math.pi/2
        yaw_rot_matrix = np.identity(4)
        yaw_rot_matrix[0][0] = math.cos(yaw_angle)
        yaw_rot_matrix[0][1] = -math.sin(yaw_angle)
        yaw_rot_matrix[1][0] = math.sin(yaw_angle)
        yaw_rot_matrix[1][1] = math.cos(yaw_angle)

        gripper_offset_mat = np.identity(4)
        gripper_offset_mat[2][3] = -gripper_offset
        gripper_offset_object_transform = np.matmul(object_transform, gripper_offset_mat)

        rotated_pose = np.matmul(np.matmul(object_transform, roll_rot_matrix), yaw_rot_matrix)

        final_pose = np.matmul(np.matmul(np.linalg.inv(base_transform), rotated_pose), gripper_offset_mat)
        final_pose_ik = np.matmul(np.matmul(np.linalg.inv(robot_world_transform), rotated_pose), gripper_offset_mat)

        pose = poseFromMatrix(final_pose_ik)
        quat = pose[:4]
        trans = pose[4:7]

        #draw = misc.DrawAxes(self.env, rotated_pose)
        urdf_str = self.get_urdf_string()
        ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
        seed_state = [0.0] * ik_solver.number_of_joints
        solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
        # if solution is not None:
        #     #put_down_pose_list.append(putdown_gripper_pose_wrt_origin)
        #     with self.env:
        #         self.robot.SetActiveDOFValues(solution)
        #     # self.set_DOFs_for_IK_solution(solution)
        # print "World"
        # print "Hello World Solution"
        #self.set_DOFs_for_IK_solution(solution)

        # gripper_link = self.robot.GetLink('gripper_l_base')
        # gripper_transform = gripper_link.GetTransform()
        # draw2 = misc.DrawAxes(self.env, gripper_transform)
        pose_list.append(final_pose_ik)
        return pose_list