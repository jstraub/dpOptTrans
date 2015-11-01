import numpy as np
from scipy.linalg import inv
from js.data.plyParse import PlyParse
import mayavi.mlab as mlab
import os.path, re
import subprocess as subp
from js.geometry.rotations import Quaternion
from js.utils.plot.colors import colorScheme
from helpers import *

cfgBuddhaRnd = {"name":"buddhaRnd", "lambdaS3": [60, 70, 80],
  "lambdaR3": 0.002}
cfgBuddha = {"name":"buddha", "lambdaS3": [80], "lambdaR3": 0.0008}
cfgBunny = {"name":"bunny", "lambdaS3": [60, 70, 80], "lambdaR3": 0.003}
cfgBunnyAB = {"name":"bunnyAB", "lambdaS3": [60, 70, 80], "lambdaR3": 0.003}
cfgBunnyZipper = {"name":"bun_zipper", "lambdaS3": [60], "lambdaR3": 0.001}
cfgEnschede = {"name":"enschede", "lambdaS3": [60, 70, 80], "lambdaR3":0.3}

cfg = cfgBunny
cfg = cfgBunnyZipper
cfg = cfgBuddha
cfg = cfgBuddhaRnd
cfg = cfgEnschede
cfg = cfgBunnyAB
loadCached = False
stopToShow = True
showUntransformed = False
applyBB = False
applyFFT = True
applyMM = False
applyICP = False
loadGlobalsolutionforICP = True
useSurfaceNormalsInICP = True

if cfg["name"] == "buddha":
  pattern = "happyStandRight_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/happy_stand/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",re.sub("happyStandRight_","",os.path.split(f)[1]))))
  gt = []
if cfg["name"] == "buddhaRnd":
  pattern = "happyStandRight_[0-9]+_angle_90_translation_0.3.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/happy_stand_rnd/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub("_angle_90_translation_0.3.ply","",re.sub("happyStandRight_","",os.path.split(f)[1]))))
  gt = []
  scans = np.roll(scans, 7)
  print scans
#  raw_input()
#  scans2  = []
#  for scan in scans:
#    id = int(re.sub("_angle_90_translation_0.3.ply","",re.sub("happyStandRight_","",os.path.split(scan)[1])))
#    if id in [264, 288]:
#      scans2.append(scan)
#  scans = scans2
if cfg["name"] == "bunny":
  pattern = "bun[0-9]+_angle_90_translation_0.3.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/bunny_rnd/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub("_angle_90_translation_0.3.ply","",re.sub("bun","",os.path.split(f)[1]))))
if cfg["name"] == "bun_zipper":
  scans = ['../data/bunny_rnd/bun_zipper.ply',
      '../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply']
  gt = ['../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply_TrueTransformation_angle_90_translation_0.3.csv']
if cfg["name"] == "bunnyAB":
  scans = ['../data/bunny_rnd/bun000_angle_90_translation_0.3.ply',
      '../data/bunny_rnd/bun045_angle_90_translation_0.3.ply']
  gt = ['../data/bunny_rnd/bun000_angle_90_translation_0.3.ply_TrueTransformation_angle_90_translation_0.3.csv',
  '../data/bunny_rnd/bun045_angle_90_translation_0.3.ply_TrueTransformation_angle_90_translation_0.3.csv']
if cfg["name"] == "enschede":
  scans = ['../data/enschede_rnd/0021770_2_inv_depth_map_gray.ply',
    '../data/enschede_rnd/0021771_2_inv_depth_map_gray_angle_50_translation_10.ply',
    '../data/enschede_rnd/0021772_2_inv_depth_map_gray_angle_50_translation_10.ply']
  gt=[]

print scans
colors = colorScheme("label")

if showUntransformed:
  figm = mlab.figure(bgcolor=(1,1,1))
  for i in range(len(scans)):
    scanPath = scans[i]
    ply = PlyParse();
    ply.parse(scanPath)
    pc = ply.getPc()
    mlab.points3d(pc[:,0], pc[:,1], pc[:,2], mode="point",
        color=colors[i%len(colors)])
  mlab.show(stop=True)

W_T_B = np.eye(4)
for i in range(1,len(scans)):
  scanApath = scans[i-1]
  scanBpath = scans[i]
  nameA = os.path.splitext(os.path.split(scanApath)[1])[0]
  nameB = os.path.splitext(os.path.split(scanBpath)[1])[0]
  transformationPath = '{}_{}.csv'.format(nameA, nameB)
  transformationPathBB = '{}_{}_BB.csv'.format(nameA, nameB)
  transformationPathICP = '{}_{}_ICP.csv'.format(nameA, nameB)
  transformationPathFFT = '{}_{}_FFT.csv'.format(nameA, nameB)
  transformationPathMM = '{}_{}_MM.csv'.format(nameA, nameB)

  if i == 1:
    plyA = PlyParse();
    plyA.parse(scanApath)
    pcA = plyA.getPc()
    figm = mlab.figure(bgcolor=(1,1,1))
    mlab.points3d(pcA[:,0], pcA[:,1], pcA[:,2], mode="point",
        color=colors[0])

  if applyBB:
    if loadCached and os.path.isfile(transformationPathBB):
      print "found transformation file and using it "+transformationPathBB
    else:
      RunBB(cfg, scanApath, scanBpath, transformationPathBB)
    transformationPath = transformationPathBB

  if applyFFT:
    if loadCached and os.path.isfile(transformationPathFFT):
      print "found transformation file and using it " +transformationPathFFT
    else:
      RunFFT(scanApath, scanBpath, transformationPathFFT)
    transformationPath = transformationPathFFT

  if applyMM:
    if loadCached and os.path.isfile(transformationPathMM):
      print "found transformation file and using it " +transformationPathMM
    else:
      RunMM(scanApath, scanBpath, transformationPathMM)
    transformationPath = transformationPathMM

  if applyICP:
    if loadCached and os.path.isfile(transformationPathICP):
      print "found transformation file and using it "+transformationPathICP
    else:
      RunICP(scanApath, scanBpath, transformationPathICP,
          useSurfaceNormalsInICP, transformationPath)
    transformationPath = transformationPathICP

  q,t = LoadTransformation(transformationPath)
  R = q.toRot().R
  print "R", R
  A_T_B = np.eye(4)
  A_T_B[:3,:3] = R.T
  A_T_B[:3,3] = -R.T.dot(t)
  W_T_B = W_T_B.dot(A_T_B)
  if i-1 < len(gt):
    q_gt,t_gt = LoadTransformation(gt[i-1])
    print "Angle to GT: {} deg".format(q.angleTo(q_gt)*180./np.pi)
    print "Translation deviation to GT: {} m".format(np.sqrt(((t-t_gt)**2).sum()))

  plyB = PlyParse();
  plyB.parse(scanBpath)
  pcB = plyB.getPc()

  R = W_T_B[:3,:3]
  t = W_T_B[:3,3]
  pcB = (1.001*R.dot(pcB.T)).T + t
  
  mlab.points3d(pcB[:,0], pcB[:,1], pcB[:,2], mode="point",
        color=colors[i%len(colors)])
  if stopToShow:
    mlab.show(stop=True)
print "Done!"
mlab.show(stop=True)
