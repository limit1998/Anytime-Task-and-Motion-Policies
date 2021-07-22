import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class DrawerGripperCloseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(DrawerGripperCloseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return DrawerGripperCloseTrajectory(self.argumnet_name)

    def apply(self,lowlevel_state,value,other_generated_values):
        robot = lowlevel_state.simulator.env.GetRobot(other_generated_values["robot"])
        taskmanip = interfaces.TaskManipulation(robot)
        if value == "close":
            drawer_name = other_generated_values["drawer"]
            drawer = lowlevel_state.simulator.env.GetKinBody(drawer_name)
            robot.Grab(drawer)
            hl_state = other_generated_values["current_hl_state"]
            for prop in hl_state.getAllProps():
                if "(in " in prop:
                    if drawer_name in prop:
                        tokens = prop.split()
                        can = lowlevel_state.simulator.env.GetKinBody(tokens[1])
                        robot.Grab(can)

    def execute(self,lowlevel_state,value,other_generated_values):
        robot = lowlevel_state.simulator.env.GetRobot(other_generated_values["robot"])
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "close":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.CloseFingers(movingdir=[-1])
                else:
                    taskmanip.CloseFingers()
                drawer_name = other_generated_values["drawer"]
                drawer = lowlevel_state.simulator.env.GetKinBody(drawer_name)
                robot.Grab(drawer)
                hl_state = other_generated_values["current_hl_state"]
                for prop in hl_state.getAllProps():
                    if "(in " in prop:
                        if drawer_name in prop:
                            tokens = prop.split()
                            can = lowlevel_state.simulator.env.GetKinBody(tokens[1])
                            robot.Grab(can)
            robot.WaitForController(0)