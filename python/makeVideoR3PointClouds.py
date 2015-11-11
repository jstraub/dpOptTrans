import numpy as np
from scipy.linalg import solve, eig
import mayavi.mlab as mlab
from project4d import *
from helpers import *
from js.geometry.rotations import *
from js.data.plyParse import PlyParse
from js.utils.plot.colors import colorScheme

colors = colorScheme("label")

cfgBunnyZipper = {"name":"bun_zipper", "lambdaS3": [60], "lambdaR3": 0.001}
cfg = cfgBunnyZipper
if cfg["name"] == "bun_zipper":
  scans = ['../data/bunny_rnd/bun_zipper.ply',
      '../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply']
  gt = ['../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply_TrueTransformation_angle_90_translation_0.3.csv']

scanApath = scans[0]
scanBpath = scans[1]
nameA = os.path.splitext(os.path.split(scanApath)[1])[0]
nameB = os.path.splitext(os.path.split(scanBpath)[1])[0]

plyA = PlyParse(); plyA.parse(scanApath); pcA = plyA.getPc()
plyB = PlyParse(); plyB.parse(scanBpath); pcB = plyB.getPc()

ranges = [-np.abs(pcA[:,0]).max()*2, np.abs(pcA[:,0]).max()*2,
    -np.abs(pcA[:,1]).max()*2, np.abs(pcA[:,1]).max()*2,
    -np.abs(pcA[:,2]).max()*2, np.abs(pcA[:,2]).max()*2]

#mlab.points3d(pcA[:,0], pcA[:,1], pcA[:,2], mode="point",
#    color=colors[0])
#mlab.view(azimuth=90, elevation=30, distance=0.6, roll=0, focalpoint=
#    np.array([ 0.0017717 ,  0.00829888, -0.00625299]))
#print mlab.view()
#mlab.show(stop=True)

path = "./bb_nodes_per_iteration_R3_bunnyFull.csv"
ts, tetras, props = LoadNodesPerIteration(path)

q_star = np.array([0.707107, 0.381209, -0.0583624, 0.592683])
q = Quaternion(q_star[0], q_star[1], q_star[2], q_star[3])

figm = mlab.figure(bgcolor=(1,1,1))
for n in range(len(ts)):
  # plot max UB rotation
  idMax = np.argmax(props[n][:,0])
  T = np.zeros((3,8))
  for j in range(8):
    T[:,j] = ts[n][tetras[n][idMax, j],:]
  t = T.sum(axis=1)/8.

  a = 2.*n/float(len(ts))*2.*np.pi
  Ry = np.array([[np.cos(a), 0, -np.sin(a)],
    [0,1,0],
    [np.sin(a),0, np.cos(a)]])
  
#  R = q.toRot().R.T
#  pcB_T = (1.001*R.dot(pcB.T)).T - R.dot(t)
#  pcA_T = pcA

  R = Ry.dot(q.toRot().R.T)
  pcB_T = (1.001*R.dot(pcB.T)).T - R.dot(t)
  pcA_T = (Ry.dot(pcA.T)).T
  
  mlab.points3d(pcA_T[:,0], pcA_T[:,1], pcA_T[:,2], mode="point",
      color=colors[0])
  mlab.points3d(pcB_T[:,0], pcB_T[:,1], pcB_T[:,2], mode="point",
        color=colors[1])
  mlab.view(azimuth=90,
      elevation=30, distance=0.9, roll=0, focalpoint=
      np.array([ 0.0017717 ,  0.00829888, -0.00625299]))
  print mlab.view()
  mlab.axes(extent=ranges)
  mlab.savefig("frame_pcR3_{:05}.png".format(n), figure=figm)
#  mlab.show(stop=True)
  mlab.clf(figm)
