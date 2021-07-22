import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class GripperCloseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(GripperCloseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return GripperCloseTrajectory(self.argumnet_name)

    def apply(self,lowlevel_state,value,other_generated_values):
        if "item" in other_generated_values:
            item = other_generated_values["item"]
        else:
            item = other_generated_values["tray"]
        robot = lowlevel_state.simulator.env.GetRobot(other_generated_values["robot"])
        taskmanip = interfaces.TaskManipulation(robot)
        if value == "close":
            robot.Grab(lowlevel_state.simulator.env.GetKinBody(item))
            if item == "t":
                next_hl_state = other_generated_values["current_hl_state"]
                for prop in next_hl_state.getAllProps():
                    if "on_tray" in prop:
                        tokens = prop.split()
                        item = tokens[1]
                        robot.Grab(lowlevel_state.simulator.env.GetKinBody(item))

    def execute(self,lowlevel_state,value,other_generated_values):
        if "item" in other_generated_values:
            item = other_generated_values["item"]
        else:
            item = other_generated_values["tray"]
        robot = lowlevel_state.simulator.env.GetRobot(other_generated_values["robot"])
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "close":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.CloseFingers(movingdir=[-1])
                else:
                    taskmanip.CloseFingers()
                robot.Grab(lowlevel_state.simulator.env.GetKinBody(item))
                if item == "t":
                    next_hl_state = other_generated_values["current_hl_state"]
                    for prop in next_hl_state.getAllProps():
                        if "on_tray" in prop:
                            tokens = prop.split()
                            item = tokens[1]
                            robot.Grab(lowlevel_state.simulator.env.GetKinBody(item))
            robot.WaitForController(0)
