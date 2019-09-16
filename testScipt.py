from openravepy import *
import Config

env = Environment()
env.Load('GeneratedEnvironments/can_world_3_cans.dae')
module = RaveCreateModule(env,'urdf')
urdf = Config.FETCH_URDF
srdf = Config.FETCH_SRDF
module.SendCommand('load '+urdf+' ' +srdf)

