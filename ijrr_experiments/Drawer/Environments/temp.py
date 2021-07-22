from openravepy import *
import math
import numpy as np

env = Environment()
env.Load("/home/naman/TMP_Merged/ijrr_experiments/Drawer/Environments/env.dae")
env.SetViewer('qtcoin')
d = env.GetKinBody("drawer")
odt = d.GetTransform()

d.SetTransform(np.eye(4))
p = odt.copy()
p[0,3] = 0
p[1,3] = 0
d.SetTransform(p)
d_aabb = d.ComputeAABB()

bottom = KinBody.GeometryInfo()
bottom._type = GeometryType.Box
bottom._vDiffuseColor = [1,0,0]
bottom._vGeomData = [d_aabb.extents()[0]-0.05,d_aabb.extents()[1],0.01]
bottom._t[2,3] = -0.2

bottom_d2 = KinBody.GeometryInfo()
bottom_d2._type = GeometryType.Box
bottom_d2._vDiffuseColor = [1,0,0]
bottom_d2._vGeomData = [d_aabb.extents()[0]-0.05,d_aabb.extents()[1],0.01]
bottom_d2._t[2,3] = -0.285

side = KinBody.GeometryInfo()
side._type = GeometryType.Box
side._vGeomData = [d_aabb.extents()[0]-0.05,0.01,0.2]
side._vDiffuseColor = [1,0,0]
side._t[1,3] = d_aabb.extents()[1] - 0.01
side._t[2,3] = -.01

side_d2 = KinBody.GeometryInfo()
side_d2._type = GeometryType.Box
side_d2._vGeomData = [d_aabb.extents()[0]-0.05,0.01,0.285]
side_d2._vDiffuseColor = [1,0,0]
side_d2._t[1,3] = d_aabb.extents()[1] - 0.01
side_d2._t[2,3] = -.01

side2 = KinBody.GeometryInfo()
side2._type = GeometryType.Box
side2._vGeomData = side._vGeomData
side2._vDiffuseColor = [1,0,0]
side2._t[1,3] = -(d_aabb.extents()[1])  + 0.01
side2._t[2,3] = -0.01


side2_d2 = KinBody.GeometryInfo()
side2_d2._type = GeometryType.Box
side2_d2._vGeomData = side_d2._vGeomData
side2_d2._vDiffuseColor = [1,0,0]
side2_d2._t[1,3] = -(d_aabb.extents()[1])  + 0.01
side2_d2._t[2,3] = -0.01

front = KinBody.GeometryInfo()
front._type = GeometryType.Box
front._vGeomData = [0.01,d_aabb.extents()[1],0.2]
front._vDiffuseColor = [1,0,0]
front._t[0,3] = -(d_aabb.extents()[0]) + 0.05
front._t[2,3] = -0.01

front_d2 = KinBody.GeometryInfo()
front_d2._type = GeometryType.Box
front_d2._vGeomData = [0.01,d_aabb.extents()[1],0.285]
front_d2._vDiffuseColor = [1,0,0]
front_d2._t[0,3] = -(d_aabb.extents()[0]) + 0.05
front_d2._t[2,3] = -0.01

back = KinBody.GeometryInfo()
back._type = GeometryType.Box
back._vGeomDataData = front._vGeomData
back._vDiffuseColor = [1,0,0]
back._t[0,3] = d_aabb.extents()[0]
back._t[2,3] = -0.01

back_d2 = KinBody.GeometryInfo()
back_d2._type = GeometryType.Box
back_d2._vGeomDataData = front_d2._vGeomData
back_d2._vDiffuseColor = [1,0,0]
back_d2._t[0,3] = d_aabb.extents()[0]
back_d2._t[2,3] = -0.01


handle = KinBody.GeometryInfo()
handle._type = GeometryType.Box
handle._vGeomData = [0.1,d_aabb.extents()[1],0.01]
handle._vDiffuseColor = [1,0,0]
handle._t[0,3]  = - (d_aabb.extents()[0]) + (0.05 / 2)
handle._t[2,3] = 0.05

handle_d2 = KinBody.GeometryInfo()
handle_d2._type = GeometryType.Box
handle_d2._vGeomData = [0.1,d_aabb.extents()[1],0.01]
handle_d2._vDiffuseColor = [1,0,0]
handle_d2._t[0,3]  = - (d_aabb.extents()[0]) + (0.05 / 2)
handle_d2._t[2,3] = 0.26


env.Remove(d)


drawer1 = RaveCreateKinBody(env,'')
drawer1.InitFromGeometries([bottom,side,side2,front,back,handle])
drawer1.SetName("top_drawer")
env.Add(drawer1)


drawer2 = RaveCreateKinBody(env,'')
drawer2.InitFromGeometries([bottom_d2,side_d2,side2_d2,front_d2,back_d2,handle_d2])
drawer2.SetName("bottom_drawer")
env.Add(drawer2)


do = env.GetKinBody("drawer_outer")
do_t = do.GetTransform()

d1p = do_t.copy()
d1p[0,3] += 0.40
d1p[2,3] += 0.30

drawer1.SetTransform(d1p)

d2p = d1p.copy()
d2p[2,3] -= 0.486
drawer2.SetTransform(d2p)


can = KinBody.GeometryInfo()
can._type = GeometryType.Cylinder
can._vGeomData = [0.03 , 0.2]
can._bVisible = True
can._vDiffuseColor = [0,1,1]


new_can = RaveCreateKinBody(env,'')
new_can.InitFromGeometries([can])
new_can.SetName("can1")

old_can = env.GetKinBody("object1")
new_can.SetTransform(old_can.GetTransform())
env.Remove(old_can)
env.Add(new_can)


import IPython
IPython.embed()