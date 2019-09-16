from openravepy import *
import Config
from Robots.Models import FetchOpenRaveRobotModel
from Robots.Models import UAVOpenRaveRobotModel
from Robots.Models import YuMiOpenRaveRobotModel
from MotionPlanners.OpenRaveMotionPlanner import *
import copy
import numpy as np

from openravepy.misc import *
from trac_ik_python.trac_ik import IK
import pdb

class OpenRaveSimulator(object):
    environment = None
    robot_init_pose = None
    def __init__(self, env_xml=None, body_name_transform_map=None):
        if OpenRaveSimulator.environment is not None:
            self.env = OpenRaveSimulator.environment
            self.AVAILABLE_MOTION_PLANNERS = ['NATIVE','OMPL_RRTConnect']
            self.robot_init_pose = OpenRaveSimulator.robot_init_pose
            # if body_name_transform_map:
            #     self.__setup_environment(body_name_transform_map)

        else:
            self.env = Environment()
            OpenRaveSimulator.environment = self.env
            # OpenRaveSimulator.environment = None
            if env_xml:
                with self.env:
                    if Config.SHOW_VIEWER:
                        self.env.SetViewer(Config.VIEWER)
                    self.env.Load(env_xml)
                self.load_robot(Config.ROBOT_NAME)
            elif body_name_transform_map:
                with self.env:
                    self.env.Load(Config.OPENRAVE_ENV_XML)
                    self.load_robot(Config.ROBOT_NAME)
                    self.__setup_environment(body_name_transform_map)
            self.openGrippers()
            self.AVAILABLE_MOTION_PLANNERS = ['NATIVE']
            self.robot_init_pose = self.env.GetRobot(Config.ROBOT_NAME).GetTransform()
            OpenRaveSimulator.robot_init_pose = self.robot_init_pose
            collision_checker = RaveCreateCollisionChecker(self.env, "pqp")
            collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
            self.env.SetCollisionChecker(collision_checker)
        self.rrt_connect_planner = OpenRave_OMPL_RRT_Connect_Planner(self.env, self.env.GetRobot(Config.ROBOT_NAME))
                # TODO create initial body_name_transform_map so that we can roll back

    def openGrippers(self):
        if Config.ROBOT_NAME == "fetch" or Config.ROBOT_NAME == "yumi":
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            taskmanip = interfaces.TaskManipulation(robot)
            with robot:
                if Config.ROBOT_NAME == 'yumi':
                    taskmanip.ReleaseFingers(movingdir=[1])
                else:
                    taskmanip.ReleaseFingers()
            robot.WaitForController(0)

    def load_robot(self, robot_name):
        if (robot_name == 'fetch'):
            RaveSetDebugLevel(DebugLevel.Error)
            FetchOpenRaveRobotModel(self.env, Config.REAL_ROBOT)
        elif robot_name == "UAV":
            RaveSetDebugLevel(DebugLevel.Error)
            UAVOpenRaveRobotModel(self.env)
        elif (robot_name == 'yumi'):
            RaveSetDebugLevel(DebugLevel.Error)
            YuMiOpenRaveRobotModel(self.env, False)

    def get_set_all_body_names(self):
        return set(bdy.GetName() for bdy in self.env.GetBodies())

    # def show_viewer(self):
    #     self.env.SetViewer('qtcoin')

    def visualize_transform(self, T):
        DrawAxes(self.env, T)

    def get_transform_matrix(self, object_name):
        return self.env.GetKinBody(object_name).GetTransform()

    def set_transform_matrix(self, object_name, transform_matrix):
        return self.env.GetKinBody(object_name).SetTransform(transform_matrix)

    def get_object(self, object_name):
        return self.env.GetKinBody(object_name)

    def get_obj_name_box_extents(self, object_name):
        geom = self.env.GetKinBody(object_name).GetLink('base').GetGeometries()[0]
        o_x, o_y, o_z = geom.GetBoxExtents().tolist()
        return o_x, o_y, o_z

        return  object.GetBoxExtents()

    def get_object_link_dimensions(self, object_name, link_name):
        with self.env:
            obj = self.get_object(object_name)
            link = obj.GetLink(link_name)
            geom = link.GetGeometries()[0]
            height = geom.GetCylinderHeight()
            base_width = 0
            base_length = 0
        return base_width, base_length, height

    def get_object_link_geometries(self, object_name, link_name):
        obj = self.get_object(object_name)
        link = obj.GetLink(link_name)
        geom = link.GetGeometries()
        return geom


    def get_matrix_from_pose(self,w, rot_x, rot_y, rot_z, translation_x, translation_y, translation_z):
        return matrixFromPose((w, rot_x, rot_y, rot_z, translation_x, translation_y, translation_z))

    def get_matrix_from_axis_angle(self,rot_x, rot_y, rot_z):
        return matrixFromAxisAngle((rot_x, rot_y, rot_z))


    def __setup_environment(self, body_name_transform_map):

        available_bodies = self.get_set_all_body_names()

        for body_name in body_name_transform_map:
            if body_name not in available_bodies:
                raise OpenRaveSimulatorException(body_name + " is not in the loaded environment xml")
            self.env.GetKinBody(body_name).SetTransform(body_name_transform_map.get(body_name))

    def apply(self, argument, other_generated_values):
        if argument.type == 'BaseTrajectory' or argument.type == "InspectTrajectory":
            if argument.value is True:
                pass
            else:
                robot = self.env.GetRobot(Config.ROBOT_NAME)
                old_dof_values = robot.GetActiveDOFIndices()
                # robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
                robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in Config.ROBOT_BASE_JOINTS])
                traj = RaveCreateTrajectory(self.env, '')
                Trajectory.deserialize(traj, argument.value)
                numWayPoints = traj.GetNumWaypoints()
                lastWayPoint = traj.GetWaypoint(numWayPoints - 1)
                lastWayPointDOFs = self.get_joint_values_from_waypoint(lastWayPoint,robot)
                robot.SetActiveDOFValues(lastWayPointDOFs)
                robot.SetActiveDOFs(old_dof_values)

        elif argument.type == "ManipTrajectory" or argument.type == "PGManipTrajectory":
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            # with robot:
            traj = RaveCreateTrajectory(self.env,'')
            Trajectory.deserialize(traj,argument.value)
                # robot.GetController().SetPath(traj)
                # import IPython
                # IPython.embed()
            numWayPoints = traj.GetNumWaypoints()
            lastWayPoint = traj.GetWaypoint(numWayPoints-1)
            lastWayPointDOFs = self.get_joint_values_from_waypoint(lastWayPoint,robot)

            robot.SetActiveDOFValues(lastWayPointDOFs)
            # robot.WaitForController(0)

        elif argument.type == 'GripperCloseTrajectory':
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            taskmanip = interfaces.TaskManipulation(robot)
            if argument.value == "close":
                # with robot:
                #     if robot.GetName() == 'yumi':
                #         taskmanip.CloseFingers(movingdir=[-1])
                #     else:
                #         taskmanip.CloseFingers()
                # robot.WaitForController(0)
                robot.Grab(self.env.GetKinBody(other_generated_values.get(other_generated_values.get('attach'))))
        elif argument.type == "GripperOpenTrajectory":
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            taskmanip = interfaces.TaskManipulation(robot)
            if argument.value == "open":
                # with robot:
                #     if robot.GetName() == 'yumi':
                #         taskmanip.ReleaseFingers(movingdir=[1])
                #     else:
                #         taskmanip.ReleaseFingers()
                # robot.WaitForController(0)
                robot.ReleaseAllGrabbed()
        return self.env


    def execute(self,arg_type,arg_value,other_generated_values,failure=False):
        # if argument.type == "robot_base_trajectory":
        #     pass
        # elif argument.type == "manipulator_trajectoryss":
        if arg_type == "BaseTrajectory":
            if arg_value is True:
                pass
            else:
                robot = self.env.GetRobot(Config.ROBOT_NAME)
                traj = RaveCreateTrajectory(self.env, '')
                Trajectory.deserialize(traj, arg_value)
                old_dof_values = robot.GetActiveDOFIndices()
                # robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
                robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in Config.ROBOT_BASE_JOINTS])
                with robot:
                    robot.GetController().SetPath(traj)
                robot.WaitForController(0)
                robot.SetActiveDOFs(old_dof_values)

        elif arg_type == "ManipTrajectory" or arg_type == "PGManipTrajectory":
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            traj = RaveCreateTrajectory(self.env, '')
            Trajectory.deserialize(traj, arg_value)
            with robot:
                robot.GetController().SetPath(traj)
            robot.WaitForController(0)
        elif arg_type == "GripperCloseTrajectory":
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            taskmanip = interfaces.TaskManipulation(robot)
            state = arg_value
            if state == "close":
                with robot:
                    if robot.GetName() == 'yumi':
                        taskmanip.CloseFingers(movingdir=[-1])
                    else:
                        taskmanip.CloseFingers()
                    robot.Grab(self.env.GetKinBody(other_generated_values[other_generated_values['attach']]))
                robot.WaitForController(0)
        elif arg_type == "GripperOpenTrajectory":
            robot = self.env.GetRobot(Config.ROBOT_NAME)
            taskmanip = interfaces.TaskManipulation(robot)
            state = arg_value
            if state == "open":
                with robot:
                    if robot.GetName() == 'yumi':
                        taskmanip.ReleaseFingers(movingdir=[1])
                    else:
                        taskmanip.ReleaseFingers()
                    robot.ReleaseAllGrabbed()
                robot.WaitForController(0)
                # import IPython
                # IPython.embed()






    def set_environment(self, values):
        with self.env:
            robots = self.env.GetRobots()
            robot = robots[0]
            if len(robots) > 0:
                robot = robots[0]

            for body in self.env.GetBodies():
                if body.IsRobot():
                    continue
                body.SetTransform(values['objects'][body.GetName()]['transform'])

            if robot is not None:
                robot.ReleaseAllGrabbed()
                robot.WaitForController(0)
                if robot.GetName() == 'yumi':
                    robot.SetTransform(values['robots'][robot.GetName()]['transform'])
                else:
                    robot.GetLink('base_link').SetTransform(values['robots'][robot.GetName()]['transform'])
                if 'dof_values' in values['robots'][robot.GetName()]:
                    robot.SetDOFValues(values['robots'][robot.GetName()]['dof_values'])
                if 'grabbed_objects' in values['robots'][robot.GetName()]:
                    for obj_name in values['robots'][robot.GetName()]['grabbed_objects']:
                        robot.Grab(self.env.GetKinBody(obj_name))
        self.env.UpdatePublishedBodies()


    def get_env_and_robot(self, ll_state):
        #TODO Support multi robots
        env = self.env

        with env:
            robots = env.GetRobots()
            robot = None
            if len(robots) > 0:
                robot = robots[0]

            for body in env.GetBodies():
                if body.IsRobot():
                    continue
                body.SetTransform(ll_state.values['objects'][body.GetName()]['transform'])

            if robot is not None:
                if robot.GetName() == 'yumi':
                    robot.SetTransform(ll_state.values['robots'][robot.GetName()]['transform'])
                else:
                    robot.GetLink('base_link').SetTransform(ll_state.values['robots'][robot.GetName()]['transform'])
                if 'dof_values' in ll_state.values['robots'][robot.GetName()]:
                    robot.SetDOFValues(ll_state.values['robots'][robot.GetName()]['dof_values'])
                if 'grabbed_objects' in ll_state.values['robots'][robot.GetName()]:
                    for obj_name in ll_state.values['robots'][robot.GetName()]['grabbed_objects']:
                        robot.Grab(env.GetKinBody(obj_name))

        return env, robot

    def get_joint_values_from_waypoint(self, waypoint,robot):
        try:
            n = robot.GetActiveDOF()
        except Exception,e:
            print dir(robot)
            print e
            n = 8
        return waypoint[0:n]

    def set_robot_active_dof_values_to_waypoint_values(self, ll_state, waypoint):
        env, _ = self.get_env_and_robot(ll_state)
        with env:
            robot = env.GetRobots()[0]
            jv = self.get_joint_values_from_waypoint(waypoint,robot)
            robot.SetActiveDOFValues(jv)
        return env

    def get_body_name_associated_with_link(self, link):
        name = link.GetParent().GetName()
        return name

    def get_collision_report(self,env):
        cr = CollisionReport()
        with env:
            robot = env.GetRobots()[0]
            env.CheckCollision(robot, cr)
        return cr




    def get_colliding_objects(self, env):
        with env:
            collision_checker = RaveCreateCollisionChecker(env, "pqp")
            collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
            env.SetCollisionChecker(collision_checker)

            robot = env.GetRobots()[0]
            bodies = env.GetBodies()
            colliding_objects = []
            for body in bodies:
                if body.IsRobot():
                    continue
                elif env.CheckCollision(robot, body):
                    colliding_objects.append(body.GetName())
            return colliding_objects


    def remove_all_removable_bodies(self, env):

        from Parser.ActionConfigParserV2 import ActionConfigParserV2
        action_config_parser = ActionConfigParserV2(Config.LL_ACTION_CONFIG)
        # print "Action Config Parser Generated"
        non_movable_bodies = action_config_parser.get_non_removable_bodies()
        # print "Got the list"

        grabbed_bodies = env.GetRobots()[0].GetGrabbed()
        # print "Got the grabbd object"
        if len(grabbed_bodies) > 0:
            grabbed_body_name = grabbed_bodies[0].GetName()
            non_movable_bodies.append(grabbed_body_name)

        # print "List Updated"
        name_to_object_and_transform = {}
        from multiprocessing import Lock
        lock = Lock()
        print env.GetBodies()
        # import IPython
        # IPython.embed()
        with env:
            for body in env.GetBodies():
                print body
                if not body.IsRobot() and body.GetName() not in non_movable_bodies:
                    body_name = body.GetName()
                    body_transform = body.GetTransform()
                    name_to_object_and_transform[body_name] = {'object': body, 'transform': body_transform}
                    print "Error at this is not possible."
                    new_body_transform = copy.deepcopy(body_transform)
                    new_body_transform[2,3] = 10
                    body.SetTransform(new_body_transform)
                    # self.env.Remove(body)
                    print "Object Removed"

        print "Objects removed and transforms stored"

        return  name_to_object_and_transform


    def get_motion_plan(self, motion_planner, robot_name, body_name_transform_map,  list_active_joint_names, list_active_joint_indices,
                        goal_transform_values, ll_state, check_collision=True , dof = False,traj_type=None,current_pose = None):
        RaveSetDebugLevel(DebugLevel.Error)

        if motion_planner not in self.AVAILABLE_MOTION_PLANNERS:
            raise OpenRaveSimulatorException(motion_planner + "is not available in chosen simulator")

        # TODO Set active manipulator by dynamically constructing a manipulator from the given joint names or values
        # TODO __setup_environment should return openrave_env instead of relying on ll_state(OpenRaveLowLevelState), ll_state should be removed
        # self.__setup_environment(body_name_transform_map)


        # self.env , _ = self.get_env_and_robot(ll_state) #TODO this should be removed

        bodies_removed = False
        name_to_object_and_transform = {}
        robot = self.env.GetRobot(Config.ROBOT_NAME)

        if check_collision == False:
            name_to_object_and_transform = self.remove_all_removable_bodies(self.env)
            bodies_removed = True

        trajectory_object = None
        if type(goal_transform_values) == list or traj_type == "base":  #TODO, find another way to dermine if it is a base pose or use an motion planner interface that is common to all sorts of motion plan problems
            # goal_transform_values = goal_transform_values[0]
            # If it is a base pose
            # transform = goal_transform_values
            # with self.env:
            #     robot = self.env.GetRobot(robot_name)
            #     robot.SetTransform(goal_transform_values)
            with self.env:
                try:
                    # robot.SetTransform(goal_transform_values)
                    # trajectory_object = True
                    # basemanip = interfaces.BaseManipulation(robot)

                    g = goal_transform_values
                    old_dof_values = robot.GetActiveDOFIndices()
                    robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in Config.ROBOT_BASE_JOINTS])
                    current_dof_vals = robot.GetActiveDOFValues()
                    if abs(np.sum(current_dof_vals - np.array(g))) < 1e-3:
                        trajectory_object = True
                    else:
                        import time
                        start_time = time.time()
                        robot.SetActiveDOFValues(current_pose)
                        trajectory_object,_,_ = self.rrt_connect_planner.get_mp_trajectory_to_goal(robot,goal_transform=g)
                        end_time = time.time()
                        print "time took:",end_time-start_time
                    robot.SetActiveDOFs(old_dof_values)
                except Exception,e:
                    print e
                    pass
        else:
            # with self.env:
            if not dof:
                if Config.IK_SOLVER == 'ik_fast':
                    iksolution = self.get_ik_solutions(robot, goal_transform_values, do_check_collisions=True)
                elif Config.IK_SOLVER == 'trac_ik':
                    iksolution = self.get_trac_ik_solution(robot, goal_transform_values, do_check_collisions=True)
            else:
                iksolution = [goal_transform_values]
            i = 0
            while i < len(iksolution):
                selected_iksolution = iksolution[i]
                try:
                    if Config.MOTION_PLANNER == Config.OPENRAVE_NATIVE_MOTION_PLANNER:
                        trajectory_object = interfaces.BaseManipulation(robot).MoveActiveJoints(selected_iksolution,outputtrajobj=True, execute=True)
                    else:
                        with self.env:
                            try:
                                trajectory_object, is_success, fail_cause = self.rrt_connect_planner.get_mp_trajectory_to_goal(robot,goal_transform=selected_iksolution)
                            except Exception,e:
                                print e
                    break

                except Exception as ex:
                    trajectory_object = None
                    import traceback
                    print(traceback.format_exc())
                    return None
                    # exit(-1)

        # self.__reset_environment()

        with self.env:
            if bodies_removed:
                for body_name in  name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])


        return trajectory_object


    def get_ik_solutions(self, robot, end_effector_transform, do_check_collisions=False):
        if do_check_collisions:
            filter_option = IkFilterOptions.CheckEnvCollisions
        else:
            filter_option = IkFilterOptions.IgnoreEndEffectorCollisions
        with self.env:
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                raveLogInfo("Generating IKModel for " + str(robot))
                ikmodel.autogenerate()
            solutions = ikmodel.manip.FindIKSolutions(end_effector_transform, filter_option)
            # DrawAxes(self.env, end_effector_transform)

        if len(solutions) == 0:
            print "NO IKs found, Probably Un-reachable transform"

        return solutions[:Config.MAX_IKs_TO_CHECK_FOR_MP]

    def get_trac_ik_solution(self, robot, end_effector_transform, do_check_collisions=False):
        if len(end_effector_transform) < 8:
            end_effector_transform = matrixFromPose(end_effector_transform)
        final_end_effector_transform = np.matmul(np.linalg.inv(robot.GetLink('world').GetTransform()),
                                                 end_effector_transform)
        urdf_str = self.get_urdf_string()
        ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
        seed_state = [0.0] * ik_solver.number_of_joints
        end_effector_pose = poseFromMatrix(end_effector_transform)
        quat = end_effector_pose[:4]
        trans = end_effector_pose[4:7]
        solutions = self.get_IK_solution(ik_solver, seed_state, trans, quat)

        if do_check_collisions:
            with self.env:
                last_DOF_state = robot.GetActiveDOFValues()
            with self.env:
                try:
                    robot.SetActiveDOFValues(solutions)
                    collision = []
                    for body in self.env.GetBodies():
                        collision.append(self.env.CheckCollision(robot, body))
                    robot.SetActiveDOFValues(last_DOF_state)
                    #print collision
                    # if collision.any():
                    #     print "Not a collision free IK"
                    #     return []
                except:
                    pass

        # import IPython
        # # IPython.embed()
        # with self.env:
        #     ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        #     if not ikmodel.load():
        #         raveLogInfo("Generating IKModel for " + str(robot))
        #         ikmodel.autogenerate()
        #     solutions = ikmodel.manip.FindIKSolutions(end_effector_transform, filter_option)
        #     # DrawAxes(self.env, end_effector_transform)
        #
        # if len(solutions) == 0:
        #     print "NO IKs found, Probably Un-reachable transform"

        return [solutions]

    def set_IK_solver(self, base, tip, urdf_str):
        ik_solver = IK(base, tip, urdf_string=urdf_str)
        return ik_solver

    def get_IK_solution(self, ik_solver, seed_state, trans, quat):
        sol = None
        while sol is None:
            try:
                sol = ik_solver.get_ik(seed_state,
                                       trans[0], trans[1], trans[2],  # X, Y, Z
                                       quat[1], quat[2], quat[3], quat[0]  # QX, QY, QZ, QW
                                       )
            except:
                print "No Solution...Retrying"
                pass
        return list(sol)

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

class OpenRaveSimulatorException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message
