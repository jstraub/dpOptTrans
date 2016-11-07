import numpy as np
from scipy.linalg import inv
from js.data.plyParse import PlyParse
import os.path, re
import subprocess as subp
from js.geometry.rotations import Quaternion
from js.utils.plot.colors import colorScheme
from helpers import *

cfgEnschede = {"name":"enschede", "lambdaS3": [60, 70, 80], "lambdaR3":0.3}
cfgBunnyZipper = {"name":"bun_zipper", "lambdaS3": [60], "lambdaR3": 0.001}
#cfgBunnyAB = {"name":"bunnyAB", "lambdaS3": [45, 60, 70, 80], "lambdaR3": 0.003}
cfgBunnyAB = {"name":"bunnyAB", "lambdaS3": [60], "lambdaR3": 0.001}
cfgBunny = {"name":"bunny", "lambdaS3": [60, 70, 80], "lambdaR3": 0.003}
cfgLymph = {"name":"lymph", "lambdaS3": [80], "lambdaR3": 1.}
cfgBuddha = {"name":"buddha", "lambdaS3": [60,70,80], "lambdaR3": 0.0008}
cfgBuddhaRnd = {"name":"buddhaRnd", "lambdaS3": [50, 60, 70, 80],
  "lambdaR3": 0.002}

cfg = cfgBunny
cfg = cfgEnschede
cfg = cfgLymph
cfg = cfgBunnyZipper
cfg = cfgBunnyAB
cfg = cfgBuddha
cfg = cfgBuddhaRnd

loadCached = False
stopToShow = False
showTransformed =  True 
showUntransformed =False
applyBB = True
applyBBEGI = False
applyFFT = False
applyMM = False
applyICP = False
runGoICP = False

simpleTranslation = True
useS3tessellation = True
useTpStessellation = not useS3tessellation and False
useAAtessellation = not useS3tessellation and not useTpStessellation

outputBoundsAt0 = True
loadGlobalsolutionforICP = True
useSurfaceNormalsInICP = True

if cfg["name"] == "lymph":
  pattern = "frame_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/lymph/dataset_3/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",re.sub("frame_","",os.path.split(f)[1]))))
  gt = []
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
#  scans = scans[:5]
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
  gt = ['../data/bunny_rnd/bun000_angle_90_translation_0.3_TrueTransformation.csv',
  '../data/bunny_rnd/bun045_angle_90_translation_0.3_TrueTransformation.csv']
  qOffset = Quaternion(w=np.cos(0.5*np.pi/4.), x=0, y=0,
      z=-np.sin(0.5**np.pi/4.)/(np.pi/4.))
if cfg["name"] == "enschede":
  scans = ['../data/enschede_rnd/0021770_2_inv_depth_map_gray.ply',
    '../data/enschede_rnd/0021771_2_inv_depth_map_gray_angle_50_translation_10.ply',
    '../data/enschede_rnd/0021772_2_inv_depth_map_gray_angle_50_translation_10.ply']
  gt=[]

print scans
colors = colorScheme("label")

if showUntransformed or showTransformed:
  import mayavi.mlab as mlab

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
  transformationPathBBEGI = '{}_{}_BBEGI.csv'.format(nameA, nameB)
  transformationPathICP = '{}_{}_ICP.csv'.format(nameA, nameB)
  transformationPathFFT = '{}_{}_FFT.csv'.format(nameA, nameB)
  transformationPathMM = '{}_{}_MM.csv'.format(nameA, nameB)
  transformationPathGoICP = '{}_{}_GoICP.csv'.format(nameA, nameB)

  if i == 1:
    plyA = PlyParse();
    plyA.parse(scanApath)
    pcA = plyA.getPc()
    if showTransformed:
      figm = mlab.figure(bgcolor=(1,1,1))
      mlab.points3d(pcA[:,0], pcA[:,1], pcA[:,2], mode="point",
          color=colors[0])

  if runGoICP:
    q,t,dt,success = RunGoICP(scanApath, scanBpath, transformationPathGoICP)

  if applyBB:
    if loadCached and os.path.isfile(transformationPathBB):
      print "found transformation file and using it "+transformationPathBB
    else:
      q,t,Ks,dt,_ = RunBB(cfg, scanApath, scanBpath,
          transformationPathBB, TpSmode=useTpStessellation,
          outputBoundsAt0=outputBoundsAt0,
          AAmode=useAAtessellation,
          simpleTranslation=simpleTranslation)
    transformationPath = transformationPathBB

  if applyBBEGI:
    if loadCached and os.path.isfile(transformationPathBBEGI):
      print "found transformation file and using it "+transformationPathBBEGI
    else:
      q,t,dt,_ = RunBB(cfg, scanApath, scanBpath,
          transformationPathBBEGI, EGImode=True, TpSmode=
          useTpStessellation)
    transformationPath = transformationPathBBEGI

  if applyFFT:
    if loadCached and os.path.isfile(transformationPathFFT):
      print "found transformation file and using it " +transformationPathFFT
    else:
      q,t,_ = RunFFT(scanApath, scanBpath, transformationPathFFT)
      with open(transformationPathFFT,'w') as f: 
        f.write("qw qx qy qz tx ty tz\n")
        f.write("{} {} {} {} {} {} {}\n".format(
          q.q[0],q.q[1],q.q[2],q.q[3],t[0],t[1],t[2]))
    transformationPath = transformationPathFFT

  if applyMM:
    if loadCached and os.path.isfile(transformationPathMM):
      print "found transformation file and using it " +transformationPathMM
    else:
      q,t,dt,_ = RunMM(scanApath, scanBpath, transformationPathMM)
    transformationPath = transformationPathMM

  if applyICP:
    if loadCached and os.path.isfile(transformationPathICP):
      print "found transformation file and using it "+transformationPathICP
    else:
      q,t,dt,_ = RunICP(scanApath, scanBpath, transformationPathICP,
          useSurfaceNormalsInICP, transformationPath)
    transformationPath = transformationPathICP

#  q,t = LoadTransformation(transformationPath)
  R = q.toRot().R
  print "R", R
  A_T_B = np.eye(4)
  A_T_B[:3,:3] = R.T
  A_T_B[:3,3] = -R.T.dot(t)
  W_T_B = W_T_B.dot(A_T_B)
  if i-1 < len(gt):
    # loaded transformation is randomX_T_X, x\in\{A,B\}
    q_gtA,t_gtA = LoadTransformation(gt[i-1])
    q_gtB,t_gtB = LoadTransformation(gt[i])
    dq_gt = q_gtB.dot(qOffset.dot(q_gtA.inverse()))
#    q_gt = q_gt.dot(qOffset)
    print 'q_offset', qOffset
    print 'q_gtA', q_gtA
    print 'q_gtB', q_gtB
    print 'dq_gt', dq_gt
    print 'q', q
    print "Angle to GT: {} deg".format(q.angleTo(dq_gt)*180./np.pi)
    print "Translation deviation to GT: {} m".format(np.sqrt(((t-t_gtA)**2).sum()))

  plyB = PlyParse();
  plyB.parse(scanBpath)
  pcB = plyB.getPc()

  R = W_T_B[:3,:3]
  t = W_T_B[:3,3]
  pcB = (1.001*R.dot(pcB.T)).T + t
  
  if showTransformed:
    mlab.points3d(pcB[:,0], pcB[:,1], pcB[:,2], mode="point",
          color=colors[i%len(colors)])
    if stopToShow:
      mlab.show(stop=True)
print "Done!"
if showTransformed or showUntransformed:
  mlab.show(stop=True)
