import copy
import OpenRaveHelper
from MotionPlanners import OpenRaveMotionPlanner
from Simulators.OpenRaveSimulator import *
import openravepy
import OpenraveUtils
import numpy as np
from Generator import Generator

import Config


class PGPMotionPlanGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        # required_values = ['pose_current', 'pose_end', 'robot_name', 'goal_joint_values', 'list_active_joint_names', 'list_active_joint_indices']

        required_values = ['pose_current', 'pose_end']
        super(PGPMotionPlanGenerator, self).__init__(known_argument_values, required_values)

        '''

        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        :param list_active_joint_names:
        :param list_active_joint_indices:
        :param goal_joint_values: list of joint values in the same order as that of list_active_joint_indices or list_active_joint_names

        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''
        self.object_name_to_transform_map = {'Unknown':'Unknown'}
        self.simulator = OpenRaveSimulator(Config.OPENRAVE_ENV_XML) #Config.SIMULATOR
        self.motion_planner = 'NATIVE'# Config.MOTION_PLANNER
        self.robot_name = Config.ROBOT_NAME
        self.robot = self.simulator.env.GetRobot(self.robot_name)
        self.list_active_joint_names = ['Unknown']
        self.list_active_joint_indices = ['Unknown']
        self.current_pose = known_argument_values.get('pose_current')
        self.goal_joint_values = known_argument_values.get('pose_end')
        self.offset = self.set_offset()
        print("pregrasp offset = ", self.offset)
        self.number_of_values = 10       
        self.step_size = self.offset / self.number_of_values
        self.ll_state = ll_state
        self.generate_function_state_err_free = self.generate_function_err_free()

    def set_offset(self):
        gf = self.robot.GetLink('gripper_link').GetTransform()
        a = self.current_pose.dot(gf)
        b = self.goal_joint_values.dot(gf)
        return (b-a)[0,3]

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
    
    def get_next_point(self,cur_pose):
        t0 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
        t1 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, self.step_size, 0, 0)
        tpgp_wrt_gp = t0.dot(t1)
        next_pose = cur_pose.dot(tpgp_wrt_gp)
        if OpenraveUtils.has_ik_to(self.simulator.env, self.simulator.env.GetRobots()[0], next_pose):
            return next_pose
        else:
            return None

    def generate_function_err_free(self):
        k = 0
        while True:
            if k > 3:
                raise StopIteration
            cpose = self.current_pose
            dof_list = []
            #Make sure first dof corresponds to initial_pose robot dof
            cur_dof = self.robot.GetActiveDOFValues()
            dof_list.append(cur_dof)
            env = self.simulator.env
            for i in range(self.number_of_values-1):
                epose = self.get_next_point(cpose)
                self.pgp_generator_fn(epose)
                cur_dof = self.robot.GetActiveDOFValues()
                dof_list.append(cur_dof)
                cpose = epose
            #Make sure final dof corresponds to goal_pose robot dof
            epose = self.goal_joint_values
            self.pgp_generator_fn(epose)
            cur_dof = self.robot.GetActiveDOFValues()
            dof_list.append(cur_dof)
            if np.allclose(epose,self.goal_joint_values):
                print("PGP to G dof points verified")
            else:
                print("PGP to G MP Failed")
                raise StopIteration
            combined_traj = openravepy.RaveCreateTrajectory(env,'')
            combined_traj.Init(self.robot.GetActiveConfigurationSpecification())
            i = 0
            for dof in dof_list:
                combined_traj.Insert(i,dof)
                i+=1
            k += 1
            openravepy.planningutils.SmoothActiveDOFTrajectory(combined_traj,self.robot)
            yield combined_traj.serialize()  



    def pgp_generator_fn(self,goal_pose):
        robot = self.simulator.env.GetRobot('fetch')
        cur_dof = robot.GetActiveDOFValues()
        manipulator = robot.GetActiveManipulator()
        manip = robot.SetActiveManipulator(manipulator)
        with self.simulator.env:  # lock environment
            Tgoal = goal_pose
            sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
            if sol is not None:
                robot.SetActiveDOFValues(sol)


    def get_next(self,extra):
        return self.generate_function_state_err_free.next()

