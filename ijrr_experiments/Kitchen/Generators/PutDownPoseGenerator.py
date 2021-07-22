import random
from src.DataStructures.Generator import Generator
import src.OpenraveUtils as OpenRaveUtils
import numpy as np
import math
from openravepy import *


class PutDownPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = []
        # required_values = ['table', 'object']


        super(PutDownPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt




    def get_next(self,flag):
        return self.generate_function_state.next()

    def _compute_put_down_pose_list(self):
        location = None
        tray = None
        item = None

        generated_values_list = []
        env = self.simulator.env
        robot = env.GetRobot(self.known_argument_values["robot"])

        case = None

        if "item" in self.known_argument_values and "tray" in self.known_argument_values:
            case = 1
        elif "item" in self.known_argument_values:
            case = 2
        else:
            case = 3

        if case == 1:
            tray_name = self.known_argument_values["tray"]
            object_name = self.known_argument_values["item"]
            i = 0
            while i < 2:
                putdonw_x = np.random.uniform(-0.2,0.25)
                putdonw_y =  np.random.uniform(-0.12,0.12)
                if "cup" in object_name:
                    z = 0.206
                else:
                    z = 0.15
                t = matrixFromPose([1,0,0,0,putdonw_x,putdonw_y,z])
                ot = env.GetKinBody(tray_name).GetTransform().dot(t)
                wrt = robot.GetLink('wrist_roll_link').GetTransform()
                it = env.GetKinBody(object_name).GetTransform()

                pose_gripper_object = np.linalg.inv(it).dot(wrt)

                updated_pdp = ot.dot(pose_gripper_object)

                if len(self.simulator.robots[self.robot_name].get_ik_solutions(updated_pdp, True)) > 0:
                    generated_values_list.append(updated_pdp)
                    i += 1

        else:
            if case == 2:
                object_name = self.known_argument_values["item"]
            else:
                object_name = self.known_argument_values["tray"]

            dest_name = "dest_" + object_name
            dest_pose = env.GetKinBody(dest_name).GetTransform()

            ot = dest_pose.copy()

            if "plate" in dest_name:
                ot[2,3] += 0.04
            elif "cup" in dest_name:
                ot[2,3] += 0.11
            elif "tray" in dest_name:
                ot[2,3] += 0.04

            wrt = robot.GetLink('wrist_roll_link').GetTransform()
            it = env.GetKinBody(object_name).GetTransform()

            pose_gripper_object = np.linalg.inv(it).dot(wrt)

            updated_pdp = ot.dot(pose_gripper_object)

            if len(self.simulator.robots[self.robot_name].get_ik_solutions(updated_pdp, True)) > 0:

                generated_values_list.append(updated_pdp)

        return generated_values_list



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
