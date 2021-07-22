from openravepy import *
import math
import numpy as np

env = Environment()
env.Load("/home/naman/TMP_Merged/ijrr_experiments/Kitchen/Environments/new_env.dae")
env.SetViewer('qtcoin')

# plate_top = KinBody.GeometryInfo()
# plate_top._type = GeometryType.Cylinder
# plate_top._vDiffuseColor = [1,0,0]
# plate_top._vGeomData = [0.12, 0.02]
# plate_top._t[2,3] += 0.035
#
# plate_base = KinBody.GeometryInfo()
# plate_base._type = GeometryType.Cylinder
# plate_base._vDiffuseColor = [1,0,0]
# plate_base._vGeomData = [0.06, 0.05]
#
can = KinBody.GeometryInfo()
can._type = GeometryType.Cylinder
can._vGeomData = [0.03 , 0.2]
can._bVisible = True
can._vDiffuseColor = [0,1,1]

#
# plate_1 = RaveCreateKinBody(env,'')
# plate_1.InitFromGeometries([plate_base,plate_top])
# plate_1.SetName("plate_1")
#
#
#
# plate_2 = RaveCreateKinBody(env,'')
# plate_2.InitFromGeometries([plate_base,plate_top])
# plate_2.SetName("plate_2")
#
#
#
# cup_1 = RaveCreateKinBody(env,'')
# cup_1.InitFromGeometries([can])
# cup_1.SetName("cup_1")
#
#
# cup_2 = RaveCreateKinBody(env,'')
# cup_2.InitFromGeometries([can])
# cup_2.SetName("cup_2")
#
# t = env.GetKinBody("cup_1").GetTransform()
# t[2,3] += 0.05
# env.Remove(env.GetKinBody("cup_1"))
# cup_1.SetTransform(t)
# env.Add(cup_1)
#
# t = env.GetKinBody("cup_2").GetTransform()
# t[2,3] += 0.05
# env.Remove(env.GetKinBody("cup_2"))
# cup_2.SetTransform(t)
# env.Add(cup_2)


b1 = KinBody.GeometryInfo()
b1._type = GeometryType.Box
b1._vGeomData = [0.05,0.05,0.005]
b1._vDiffuseColor = [1,0,0]
#
#
# b2 = KinBody.GeometryInfo()
# b2._type = GeometryType.Box
# b2._vGeomData = [0.05,0.05,0.075]
# b2._vDiffuseColor = [0,1,0]
#
# b3 = KinBody.GeometryInfo()
# b3._type = GeometryType.Box
# b3._vGeomData = [0.25,0.12,0.015]
# b3._vDiffuseColor = [0,1,0]
# b3._t[2,3] += 0.09
#
#
#
m5 = RaveCreateKinBody(env,'')
m5.InitFromGeometries([b1])
m5.SetName("t_table_1")
#
# tray = RaveCreateKinBody(env,'')
# tray.InitFromGeometries([b2,b3])
# tray.SetName("t")
#
#
# p1t = env.GetKinBody("plate_1").GetTransform()
# tt = p1t.copy()
# tt[0,3] = -0.4
# tray.SetTransform(tt)
# env.Add(tray)
#
# m1t = env.GetKinBody("dest_plate_1").GetTransform()
# tt = m1t.copy()
# tt[1,3] -= 0.9
# m5.SetTransform(tt)
# env.Add(m5)




import IPython
IPython.embed()