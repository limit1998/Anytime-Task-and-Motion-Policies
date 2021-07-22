import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class BaseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(BaseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return BaseTrajectory(self.argumnet_name)

    def apply(self,ll_state,value,other_generated_values):
        if value is True:
            pass
        else:
            simulator = ll_state.simulator
            robot = ll_state.simulator.env.GetRobot(other_generated_values["agent"])
            # old_dof_values = robot.GetActiveDOFIndices()
            traj = RaveCreateTrajectory(simulator.env, '')
            Trajectory.deserialize(traj, value)
            numWayPoints = traj.GetNumWaypoints()
            lastWayPoint = traj.GetWaypoint(numWayPoints - 1)
            lastWayPointDOFs = simulator.get_joint_values_from_waypoint(lastWayPoint, robot)
            robot.SetActiveDOFValues(lastWayPointDOFs)
            # robot.SetActiveDOFs(old_dof_values)

    def execute(self,ll_state,value,other_generated_values):
        if value is True:
            pass
        else:
            simulator = ll_state.simulator
            robot = ll_state.simulator.env.GetRobot(other_generated_values["agent"])
            traj = RaveCreateTrajectory(simulator.env, '')
            Trajectory.deserialize(traj, value)
            with robot:
                robot.GetController().SetPath(traj)
            robot.WaitForController(0)