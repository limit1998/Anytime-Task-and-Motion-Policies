import random
from Generator import Generator
import OpenraveUtils
import numpy as np
from openravepy import *
import math
import copy
import Config
from trac_ik_python.trac_ik import IK



class PutDownPoseGeneratorKeva(Generator):
    root_plank = ''

    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['robot', 'plank1', 'region']

        super(PutDownPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()
        self.table_name = 'table60'
        self.object_name = known_argument_values.get('plank1')
        self.putdown_side = known_argument_values.get('region')
        self.reference_structure_path = Config.REFERENCE_STRUCTURE_PATH


    def reset(self):
        if self.object_name == PutDownPoseGeneratorKeva.root_plank:
            PutDownPoseGeneratorKeva.root_plank = ""
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt

    def get_next(self,flag):
        return self.generate_function_state.next()

    def set_IK_solver(self, base, tip, urdf_str):
        ik_solver = IK(base, tip, urdf_string=urdf_str)
        return ik_solver

    def get_IK_solution(self, ik_solver, seed_state, trans, quat):
        max_count = 20
        count = 0
        while True and count < max_count:
            sol = ik_solver.get_ik(seed_state,
                                       trans[0], trans[1], trans[2],  # X, Y, Z
                                       quat[1], quat[2], quat[3], quat[0]  # QX, QY, QZ, QW
                                       )
            if sol is None:
                print "No Solution...Retrying"
                count +=1
            else:
                print "IK Solution Found"
                return list(sol)
        print "NO IK Found...Unreachable Transform"
        return sol

    def set_DOFs_for_IK_solution(self, solution):
        with self.simulator.env:
            try:
                self.robot.SetActiveDOFValues(solution)
                print "Hello World"
            except:
                pass

    def get_urdf_string(self):
        with open(Config.FETCH_URDF, 'r') as file:
            urdf_str = file.read()

        return urdf_str

    def _compute_put_down_pose_list(self):
        global root_plank
        print self.known_argument_values
        number_poses = 10
        plank_length = 0.117475
        min_dist = plank_length/3.0
        max_dist = plank_length


        env = self.simulator.env
        table = env.GetKinBody(self.table_name)
        table_transform = table.GetTransform()
        table_surface_transform = copy.deepcopy(table_transform)
        table_surface_transform[2][3] += 0.1
        robot = env.GetRobots()[0]

        manipulator = robot.GetActiveManipulator()
        manip = robot.SetActiveManipulator(manipulator)  # set the manipulator to leftarm

        table_geom = env.GetKinBody(self.table_name).GetLink('base').GetGeometries()[0]
        table_x, table_y, table_z = table_geom.GetBoxExtents().tolist()

        robot_world_transform = robot.GetLink('world').GetTransform()

        gripper_base_link = robot.GetLink('gripper_l_base')
        gripper_base_transform = gripper_base_link.GetTransform()
        #wrist_roll_link = robot.GetLink('wrist_roll_link')
        #wrist_pose_wrt_origin = wrist_roll_link.GetTransform()
        try:
            grabbed_plank_pose = robot.GetGrabbed()[0].GetTransform()
        except:
            import IPython
            IPython.embed()
        plank_pose_wrt_gripper = np.linalg.inv(np.matmul(np.linalg.inv(grabbed_plank_pose), gripper_base_transform))

        plank_contat_translation = np.identity(4)
        plank_contat_translation[0][3] = -plank_length/2.0
        plank_contact_pose_wrt_origin = np.matmul(grabbed_plank_pose, plank_contat_translation)
        # a = misc.DrawAxes(env, gripper_base_transform)
        # ax = misc.DrawAxes(env, grabbed_plank_pose)
        # ax1 = misc.DrawAxes(env, plank_contact_pose_wrt_origin)
        plank_contact_pose_wrt_plank_base = np.linalg.inv(np.matmul(np.linalg.inv(plank_contact_pose_wrt_origin), grabbed_plank_pose))
        gripper_pose_wrt_plank_contact = np.linalg.inv(np.matmul(plank_pose_wrt_gripper, plank_contact_pose_wrt_plank_base))

        x_limit = table_x - 0.1
        y_limit = table_y - 0.2

        putdown_x = []
        putdown_y = []


        reference_env = Environment()
        reference_env.Load(self.reference_structure_path)

        translation_matrix = np.identity(4)

        pitch_angle = -math.pi / 2.0
        roll_angle = math.pi / 2.0

        pitch_rotation_matrix = np.identity(4)
        roll_rotation_matrix = np.identity(4)

        pitch_rotation_matrix[0][0] = math.cos(pitch_angle)
        pitch_rotation_matrix[0][2] = math.sin(pitch_angle)
        pitch_rotation_matrix[2][0] = -math.sin(pitch_angle)
        pitch_rotation_matrix[2][2] = math.cos(pitch_angle)

        roll_rotation_matrix[1][1] = math.cos(roll_angle)
        roll_rotation_matrix[1][2] = -math.sin(roll_angle)
        roll_rotation_matrix[2][1] = math.sin(roll_angle)
        roll_rotation_matrix[2][2] = math.cos(roll_angle)

        rotation_matrix = np.matmul(pitch_rotation_matrix, roll_rotation_matrix)

        put_down_pose_list = []

        if len(root_plank) == 0:
            for i in range(number_poses):
                putdown_x.append(random.uniform(-x_limit+0.1, -x_limit+0.2))
                putdown_y.append(random.uniform(0, y_limit))

            for x, y in zip(putdown_x, putdown_y):
                translation_matrix[0][3] = x
                translation_matrix[1][3] = y

                put_down_pose = np.matmul(np.matmul(table_surface_transform, translation_matrix), rotation_matrix)
                put_down_pose_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), put_down_pose)

                putdown_gripper_pose_wrt_origin = np.matmul(put_down_pose_wrt_robot_world, gripper_pose_wrt_plank_contact)
                roll_angle = math.pi
                roll_rot_matrix = np.identity(4)
                roll_rot_matrix[1][1] = math.cos(roll_angle)
                roll_rot_matrix[1][2] = -math.sin(roll_angle)
                roll_rot_matrix[2][1] = math.sin(roll_angle)
                roll_rot_matrix[2][2] = math.cos(roll_angle)

                yaw_angle = -math.pi / 2
                yaw_rot_matrix = np.identity(4)
                yaw_rot_matrix[0][0] = math.cos(yaw_angle)
                yaw_rot_matrix[0][1] = -math.sin(yaw_angle)
                yaw_rot_matrix[1][0] = math.sin(yaw_angle)
                yaw_rot_matrix[1][1] = math.cos(yaw_angle)
                final_put_down_pose = np.matmul(np.matmul(putdown_gripper_pose_wrt_origin, roll_rot_matrix), yaw_rot_matrix)


                pose = poseFromMatrix(putdown_gripper_pose_wrt_origin)
                quat = pose[:4]
                trans = pose[4:7]

                # draw = misc.DrawAxes(self.env, rotated_pose)
                urdf_str = self.get_urdf_string()
                ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
                seed_state = [10.0] * ik_solver.number_of_joints
                solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
                if solution is not None:
                    put_down_pose_list.append(putdown_gripper_pose_wrt_origin)
                #     with env:
                #         robot.SetActiveDOFValues(solution)
                # print "World"
                # print "Hello World Solution"


            root_plank = self.object_name

        else:
            reference_root_plank_transform = reference_env.GetKinBody(root_plank).GetTransform()
            reference_current_plank_transform = reference_env.GetKinBody(self.object_name).GetTransform()
            reference_current_plank_wrt_root_plank = np.matmul(np.linalg.inv(reference_root_plank_transform), reference_current_plank_transform)
            root_plank_transform = env.GetKinBody(root_plank).GetTransform()


            put_down_pose = np.matmul(root_plank_transform, reference_current_plank_wrt_root_plank)
            put_down_pose_wrt_robot_world = np.matmul(np.linalg.inv(robot_world_transform), put_down_pose)

            #putdown_gripper_pose_wrt_origin = np.matmul(put_down_pose, np.linalg.inv(plank_pose_wrt_gripper))
            putdown_gripper_pose_wrt_origin = np.matmul(put_down_pose_wrt_robot_world, np.linalg.inv(plank_pose_wrt_gripper))


            pose = poseFromMatrix(putdown_gripper_pose_wrt_origin)
            quat = pose[:4]
            trans = pose[4:7]

            # draw = misc.DrawAxes(self.env, rotated_pose)
            urdf_str = self.get_urdf_string()
            ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
            seed_state = [0.0] * ik_solver.number_of_joints
            solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
            if solution is not None:
                #self.set_DOFs_for_IK_solution(solution)
                put_down_pose_list.append(putdown_gripper_pose_wrt_origin)
                with env:
                    robot.SetActiveDOFValues(solution)
                env.UpdatePublishedBodies()
            print "World"
            print "Hello World Solution"

        random.shuffle(put_down_pose_list)
        return put_down_pose_list





def generate_pre_grasp_pose(openrave_ll_state, object_name, grasp_pose):
    env, robot = openrave_ll_state.get_env_and_robot()
    with env:
        obj = env.GetKinBody(object_name)
        orig_t = obj.GetTransform()
        gp_at_origin = grasp_pose.dot(np.linalg.inv(orig_t))

        approach_dist = 0.03
        t4 = matrixFromPose((1, 0, 0, 0, -approach_dist, 0, 0))
        pre_grasp_pose = gp_at_origin.dot(t4)
        pre_grasp_pose_wrt_obj = pre_grasp_pose.dot(orig_t)
    return [pre_grasp_pose_wrt_obj]
