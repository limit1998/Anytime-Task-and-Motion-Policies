# from openravepy import *
# from aair_fetch import FetchOpenRaveRobotModel
#
#
# def parseXML(openrave_env_xml, robot_name ='fetch'):
#
#         env = Environment()
#         env.Load(openrave_env_xml)
#         if (robot_name == 'fetch'):
#             robotModel = FetchOpenRaveRobotModel.FetchOpenRaveRobotModel(env, False)
#
#         env_json= {}
#         robots = {}
#         objects = {}
#
#         env_json['xml'] = openrave_env_xml
#
#         for body in env.GetBodies():
#             transform = body.GetTransform()
#             name = body.GetName()
#             if body.IsRobot():
#                 assert name not in robots, "ERROR, Openrave env_json has duplicate robot names"
#                 robots[name] = {}
#                 robots[name]['transform'] = transform
#
#             else:
#                 assert name not in objects, "ERROR, Openrave env_json has duplicate object names"
#                 objects[name] = {}
#                 objects[name]['transform'] = transform
#
#         env_json['robots'] = robots
#         env_json['objects'] = objects
#
#         return env_json
#
#
# def get_json(env, openrave_env_xml):
#     env_json = {}
#     robots = {}
#     objects = {}
#
#     env_json['xml'] = openrave_env_xml
#     with env:
#         for body in env.GetBodies():
#             transform = body.GetTransform()
#             name = body.GetName()
#             if body.IsRobot():
#                 assert name not in robots, "ERROR, Openrave env_json has duplicate robot names"
#                 robot = env.GetRobot(name)
#                 robots[name] = {}
#                 robots[name]['transform'] = transform
#                 robots[name]['dof_values'] = robot.GetDOFValues()
#                 if len(robot.GetGrabbed()) > 0:
#                     robots[name]['grabbed_objects'] = [o.GetName() for o in robot.GetGrabbed()]
#
#             else:
#                 assert name not in objects, "ERROR, Openrave env_json has duplicate object names"
#                 objects[name] = {}
#                 objects[name]['transform'] = transform
#
#     env_json['robots'] = robots
#     env_json['objects'] = objects
#
#     return env_json
