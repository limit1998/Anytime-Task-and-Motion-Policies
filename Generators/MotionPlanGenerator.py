import copy
import OpenRaveHelper
from MotionPlanners import OpenRaveMotionPlanner
from Simulators.OpenRaveSimulator import *



from Generator import Generator

import Config


class MotionPlanGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        # required_values = ['pose_current', 'pose_end', 'robot_name', 'goal_joint_values', 'list_active_joint_names', 'list_active_joint_indices']

        required_values = ['pose_current', 'pose_end']
        super(MotionPlanGenerator, self).__init__(known_argument_values, required_values)

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
        self.ll_state = ll_state
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        self.number_of_values = 10


    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

        
    def generate_function_err(self):
        for mp_plan_no in range(1):
            end_dof = False
            if len(list(self.goal_joint_values)) == 8:
                end_dof = True
            mp = self.simulator.get_motion_plan(self.motion_planner, self.robot_name,
                                                self.object_name_to_transform_map,
                                                self.list_active_joint_names, self.list_active_joint_indices,
                                                self.goal_joint_values,
                                                self.ll_state,
                                                check_collision=False,
                                                dof = end_dof,
                                                current_pose = self.current_pose
                                                )
            if mp is None:
                raise StopIteration
            yield mp
        raise StopIteration




    def generate_function_err_free(self):
        for mp_plan_no in range(1):
            end_dof = False
            if len(list(self.goal_joint_values)) == 8:
                end_dof = True
            for attempts in range(5):
                 mp = self.simulator.get_motion_plan(self.motion_planner, self.robot_name, self.object_name_to_transform_map,
                                               self.list_active_joint_names, self.list_active_joint_indices,
                                               self.goal_joint_values,
                                               self.ll_state,dof = end_dof,
                                                current_pose = self.current_pose)
                 if mp is not None:
                    break;
            if mp is None:
                raise StopIteration

            yield mp
        raise StopIteration

    def get_next(self,extra):
        if extra["flag"]:
            return self.generate_function_state_err_free.next()
        else:
            return self.generate_function_state_err.next()

