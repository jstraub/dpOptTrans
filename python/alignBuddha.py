import numpy as np
from scipy.linalg import inv
from js.data.plyParse import PlyParse
import os.path, re
import subprocess as subp
from js.geometry.rotations import Quaternion
from js.utils.plot.colors import colorScheme
from helpers import *

def logDeviations(fRes,pathGtA,pathGtB,q_ba,t_ba,dt,algo):
  # loaded transformation is randomX_T_X, x\in\{A,B\}
  q_gtA,t_gtA,_ = LoadTransformation(pathGtA)
  q_gtB,t_gtB,_ = LoadTransformation(pathGtB)
  dq_gt = q_gtA.dot(q_gtB.inverse())
  dt_gt = t_gtA - dq_gt.rotate(t_gtB)
  dAngDeg = q_ba.angleTo(dq_gt)*180./np.pi
  dTrans = np.sqrt(((t_ba-dt_gt)**2).sum())
  print 'q_gtA', q_gtA
  print 'q_gtB', q_gtB
  print 'dq_gt', dq_gt
  print 'dt_gt', dt_gt
  print 'q_ba', q_ba
  print 't_ba', t_ba
  print "Angle to GT: {} deg".format(dAngDeg)
  print "Translation deviation to GT: {} m".format(dTrans)
  fRes.write("{} {} {} {} {} {}\n".format(algo,i-1,i,dAngDeg,dTrans,dt))
  fRes.flush()
  print "wrote results to " + fRes.name

cfgEnschede = {"name":"enschede", "lambdaS3": [60, 70, 80], "lambdaR3":0.3}
cfgBunnyZipper = {"name":"bun_zipper", "lambdaS3": [60], "lambdaR3":
    0.001, "maxLvlR3":15, "maxLvlS3":5 }
#cfgBunnyAB = {"name":"bunnyAB", "lambdaS3": [45, 60, 70, 80], "lambdaR3": 0.003}
cfgBunnyAB = {"name":"bunnyAB", "lambdaS3":
    [60,70,80], "lambdaR3": 0.001, "maxLvlR3":15, "maxLvlS3":15}
cfgBunny = {"name":"bunny", "lambdaS3": [60, 70, 80], "lambdaR3": 0.001,
    "maxLvlR3":15, "maxLvlS3":5}
cfgLymph = {"name":"lymph", "lambdaS3": [80], "lambdaR3": 1.}
cfgBuddha = {"name":"buddha", "lambdaS3": [60,70,80], "lambdaR3": 0.0008}
cfgBuddhaRnd = {"name":"buddhaRnd", "lambdaS3": [50,60,70,80],
  "lambdaR3": 0.002}
cfgBuddhaRnd = {"name":"buddhaRnd", "lambdaS3": [60,70,80], "lambdaR3": 0.002, 
    "maxLvlR3":15, "maxLvlS3":5}
# lambdaR3 10 was good; lambdaS3 ,65,80
cfgWood= {"name":"wood", "lambdaS3": [65], "lambdaR3": 10., 
    "maxLvlR3":8, "maxLvlS3":14}
cfgApartment= {"name":"apartment", "lambdaS3": [45,65,80], "lambdaR3": 1., 
    "maxLvlR3":10, "maxLvlS3":14, "icpCutoff": 0.1}

#cfgDesk1 = {"name":"desk1", "lambdaS3": [60,70,80], "lambdaR3": 0.1, 
cfgDesk1 = {"name":"desk1", "lambdaS3": [45,65,85], "lambdaR3": 0.15, 
    "maxLvlR3":14, "maxLvlS3":14, "icpCutoff": 0.1}

#fast?
cfgStairs = {"name":"stairs", "lambdaS3": [45], "lambdaR3": 15.,
    "maxLvlR3":14, "maxLvlS3":14 } #14
# accurate?
cfgStairs = {"name":"stairs", "lambdaS3": [45,65,80], "lambdaR3": 15.,
    "maxLvlR3":14, "maxLvlS3":14 } #14
cfgStairs = {"name":"stairs", "lambdaS3": [45,65], "lambdaR3": 15.,
    "maxLvlR3":14, "maxLvlS3":14 } #14

# accurate?
cfgD458fromDesk= {"name":"D458fromDesk", "lambdaS3": [45,65,85], "lambdaR3": 0.15, 
    "maxLvlR3":14, "maxLvlS3":14, "icpCutoff": 0.1}

#fast? fails to align randys desk
cfgD458fromDesk= {"name":"D458fromDesk", "lambdaS3": [45,65,85], "lambdaR3": 0.5, 
    "maxLvlR3":14, "maxLvlS3":14, "icpCutoff": 0.1}

cfg = cfgEnschede
cfg = cfgLymph
cfg = cfgBuddha
cfg = cfgBunny
cfg = cfgBunnyZipper
cfg = cfgBunnyAB
cfg = cfgWood

cfg = cfgApartment
cfg = cfgStairs
cfg = cfgDesk1

cfg = cfgBuddhaRnd
cfg = cfgApartment

loadCached = False
stopToShow = True
stopEveryI = 1
showTransformed =  True 
showUntransformed =False

applyBB = False
applyBBEGI = False
applyFFT = False
applyMM = False
runGoICP = False
runGogma = True
applyICP = False

tryMfAmbig = False
if cfg["name"] == "stairs":
  tryMfAmbig = True

simpleTranslation = False
simpleRotation = False
useS3tessellation = True
useTpStessellation = not useS3tessellation and False
useAAtessellation = not useS3tessellation and not useTpStessellation

outputBoundsAt0 = True
loadGlobalsolutionforICP = True
useSurfaceNormalsInICP = True

qOffset = Quaternion()

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
    int(re.sub("_angle_90_translation_0.3.ply","",
      re.sub("happyStandRight_","",os.path.split(f)[1]))))
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
  gt = []
if cfg["name"] == "bun_zipper":
  scans = ['../data/bunny_rnd/bun_zipper.ply',
      '../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply']
  #gt = ['../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply_TrueTransformation_angle_90_translation_0.3.csv']
  gt = []
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
if cfg["name"] == "stairs":
  pattern = "HokuyoPcNormals_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/stairs/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",
      re.sub("HokuyoPcNormals_","",os.path.split(f)[1]))))
#  scans = [
#      os.path.abspath('../data/stairs/HokuyoPcNormals_1.ply'),
#      os.path.abspath('../data/stairs/HokuyoPcNormals_2.ply')]
  gt=[]
  pattern = "pose_[0-9]+.csv$"
  for root, dirs, files in os.walk("../data/stairs/"):
    for f in files:
      if re.search(pattern, f):
        gt.append(os.path.join(root, f))
  gt = sorted(gt, key=lambda f: 
    int(re.sub(".csv","",
      re.sub("pose_","",os.path.split(f)[1]))))
#  scans = scans[:3]
#  gt = gt[:3]
  print scans
  print gt 
if cfg["name"] == "apartment":
  pattern = "HokuyoPcNormals_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/apartment/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",
      re.sub("HokuyoPcNormals_","",os.path.split(f)[1]))))
  gt=[]
  pattern = "pose_[0-9]+.csv$"
  for root, dirs, files in os.walk("../data/apartment/"):
    for f in files:
      if re.search(pattern, f):
        gt.append(os.path.join(root, f))
  gt = sorted(gt, key=lambda f: 
    int(re.sub(".csv","",
      re.sub("pose_","",os.path.split(f)[1]))))
  print scans
  print gt
if cfg["name"] == "wood":
  pattern = "HokuyoPcNormals_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("../data/wood_summer/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",
      re.sub("HokuyoPcNormals_","",os.path.split(f)[1]))))
  gt=[]
  pattern = "pose_[0-9]+.csv$"
  for root, dirs, files in os.walk("../data/wood_summer/"):
    for f in files:
      if re.search(pattern, f):
        gt.append(os.path.join(root, f))
  gt = sorted(gt, key=lambda f: 
    int(re.sub(".csv","",
      re.sub("pose_","",os.path.split(f)[1]))))
  print scans
  print gt
if cfg["name"] == "desk1":
  pattern = "frame_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("/data/vision/fisher/expres1/jstraub/optRotTransCVPR2017_KFs/desk1/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",
      re.sub("frame_","",os.path.split(f)[1]))))
#  scans = scans[:4]
  print scans
#  scans = [
#      os.path.abspath('../data/stairs/HokuyoPcNormals_1.ply'),
#      os.path.abspath('../data/stairs/HokuyoPcNormals_2.ply')]
  gt=[]
if cfg["name"] == "D458fromDesk":
  pattern = "frame_[0-9]+.ply$"
  scans = []
  for root, dirs, files in os.walk("/data/vision/fisher/expres1/jstraub/optRotTransCVPR2017_KFs/32-D458_fromDesk/"):
    for f in files:
      if re.search(pattern, f):
        scans.append(os.path.join(root, f))
  scans = sorted(scans, key=lambda f: 
    int(re.sub(".ply","",
      re.sub("frame_","",os.path.split(f)[1]))))
#  scans = scans[:4]
  print scans
#  scans = [
#      os.path.abspath('../data/stairs/HokuyoPcNormals_1.ply'),
#      os.path.abspath('../data/stairs/HokuyoPcNormals_2.ply')]
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
    n = ply.n
    #    mlab.points3d(pc[:,0], pc[:,1], pc[:,2], mode="point",
#        color=colors[i%len(colors)])
    mlab.points3d(n[:,0], n[:,1], n[:,2], mode="point",
        color=colors[i%len(colors)])
  mlab.show(stop=True)

prefix = "{}_{}".format(cfg["name"],int(np.floor(time.time()*1e3)))

if len(gt) > 0:
  fRes = open(prefix+"resultsVsGrountruth.csv","w")
  fRes.write("algo idFrom idTo dAngDeg dTrans dTimeSec\n")

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
  transformationPathGogma = '{}_{}_Gogma.csv'.format(nameA, nameB)

  if i == 1:
    plyA = PlyParse();
    plyA.parse(scanApath)
    pcA = plyA.getPc()
    if showTransformed:
      figm = mlab.figure(bgcolor=(1,1,1))
      mlab.points3d(pcA[:,0], pcA[:,1], pcA[:,2], mode="point",
          color=colors[0])

  if runGogma:
    q_ba,t_ba,dt,success = RunGogma(scanApath, scanBpath, transformationPathGogma)
    if i-1 < len(gt):
      logDeviations(fRes, gt[i-1], gt[i], q_ba,t_ba,dt,"GOGMA")

  if runGoICP:
    q_ba,t_ba,dt,success = RunGoICP(scanApath, scanBpath, transformationPathGoICP)

    if i-1 < len(gt):
      logDeviations(fRes, gt[i-1], gt[i], q_ba,t_ba,dt,"GoICP")

  if applyBB:
    if loadCached and os.path.isfile(transformationPathBB):
      print "found transformation file and using it "+transformationPathBB
    else:
      q_ba,t_ba,Ks,dt,_ = RunBB(cfg, scanApath, scanBpath,
          transformationPathBB, TpSmode=useTpStessellation,
          outputBoundsAt0=outputBoundsAt0,
          AAmode=useAAtessellation,
          simpleTranslation=simpleTranslation,
          simpleRotation=simpleRotation,
          tryMfAmbig=tryMfAmbig)
    transformationPath = transformationPathBB

    if i-1 < len(gt):
      logDeviations(fRes, gt[i-1], gt[i], q_ba,t_ba,dt,"BB")

  if applyBBEGI:
    if loadCached and os.path.isfile(transformationPathBBEGI):
      print "found transformation file and using it "+transformationPathBBEGI
    else:
      q_ba,t_ba,dt,_ = RunBB(cfg, scanApath, scanBpath,
          transformationPathBBEGI, EGImode=True, TpSmode=
          useTpStessellation)
    transformationPath = transformationPathBBEGI

  if applyFFT:
    if loadCached and os.path.isfile(transformationPathFFT):
      print "found transformation file and using it " +transformationPathFFT
    else:
      q_ba,t_ba,dt,_ = RunFFT(scanApath, scanBpath, transformationPathFFT)
      with open(transformationPathFFT,'w') as f: 
        f.write("qw qx qy qz tx ty tz\n")
        f.write("{} {} {} {} {} {} {}\n".format(
          q_ba.q[0],q_ba.q[1],q_ba.q[2],q_ba.q[3],t_ba[0],t_ba[1],t_ba[2]))
    transformationPath = transformationPathFFT

    if i-1 < len(gt):
      logDeviations(fRes, gt[i-1], gt[i], q_ba,t_ba,dt,"FFT")

  if applyMM:
    if loadCached and os.path.isfile(transformationPathMM):
      print "found transformation file and using it " +transformationPathMM
    else:
      q_ba,t_ba,dt,_ = RunMM(scanApath, scanBpath, transformationPathMM)
    transformationPath = transformationPathMM

  if applyICP:
    if loadCached and os.path.isfile(transformationPathICP):
      print "found transformation file and using it "+transformationPathICP
    else:
      if "icpCutoff" in cfg:
        q_ba,t_ba,dt,_ = RunICP(scanApath, scanBpath, transformationPathICP,
            useSurfaceNormalsInICP, transformationPath,
            cutoff=cfg["icpCutoff"])
      else:
        q_ba,t_ba,dt,_ = RunICP(scanApath, scanBpath, transformationPathICP,
            useSurfaceNormalsInICP, transformationPath)
    transformationPath = transformationPathICP
    if i-1 < len(gt):
      logDeviations(fRes, gt[i-1], gt[i], q_ba,t_ba,dt, "ICP")

#  q_ba,t_ba = LoadTransformation(transformationPath)
  R_ba = q_ba.toRot().R
  print "R_ba", R_ba
  A_T_B = np.eye(4)
  A_T_B[:3,:3] = R_ba.T
  A_T_B[:3,3] = -R_ba.T.dot(t_ba)
  W_T_B = W_T_B.dot(A_T_B)

  if showTransformed:
    plyB = PlyParse();
    plyB.parse(scanBpath)
    pcB = plyB.getPc()
    R_wb = W_T_B[:3,:3]
    t_wb = W_T_B[:3,3]
    pcB = (1.001*R_wb.dot(pcB.T)).T + t_wb
    mlab.points3d(pcB[:,0], pcB[:,1], pcB[:,2], mode="point",
          color=colors[i%len(colors)])
    if stopToShow and i%stopEveryI == 0:
      mlab.show(stop=True)
print "Done!"
if len(gt) > 0:
  fRes.close()
if showTransformed or showUntransformed:
  mlab.show(stop=True)
