import numpy as np
from scipy.linalg import inv
from js.data.plyParse import PlyParse
import mayavi.mlab as mlab
import os.path, re
import subprocess as subp
from js.geometry.rotations import Quaternion
from js.utils.plot.colors import colorScheme

def LoadTransformation(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    q = qt[:4];
    print 'q', q
    t = qt[4:7];
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
  return q,t

def LoadTransformationAndBounds(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    q = qt[:4];
    print 'q', q
    t = qt[4:7];
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    lbS3 = qt[7]
    lbR3 = qt[8]
  return q,t, lbS3, lbR3

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
cfg = cfgBunnyAB
cfg = cfgEnschede
loadCached = True
stopToShow = True
showUntransformed = False
applyBB = True
applyEGI = False
applyMM = False
applyICP = True
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
  transformationPathICP = '{}_{}_ICP.csv'.format(nameA, nameB)
  transformationPathEGI = '{}_{}_EGI.csv'.format(nameA, nameB)
  transformationPathMM = '{}_{}_MM.csv'.format(nameA, nameB)

  if i == 1:
    plyA = PlyParse();
    plyA.parse(scanApath)
    pcA = plyA.getPc()
    figm = mlab.figure(bgcolor=(1,1,1))
    mlab.points3d(pcA[:,0], pcA[:,1], pcA[:,2], mode="point",
        color=colors[0])

  if applyBB:
    if loadCached and os.path.isfile(transformationPath):
      print "found transformation file and using it "+transformationPath
    else:
      lbsS3 = np.zeros(len(cfg["lambdaS3"]))
      lbsR3 = np.zeros(len(cfg["lambdaS3"]))
      qs, ts = [],[]
      for j,lambdaS3 in enumerate(cfg["lambdaS3"]):
        args = ['../pod-build/bin/dpvMFoptRotPly', 
            '-a {}'.format(scanApath), 
            '-b {}'.format(scanBpath), 
            '-l {}'.format(lambdaS3),
            '-t {}'.format(cfg["lambdaR3"]),
            '-o {}_{}'.format(nameA, nameB)
            ]
        print " ".join(args)
        err = subp.call(" ".join(args), shell=True)
        q,t,lbsS3[j],lbsR3[j] = LoadTransformationAndBounds(transformationPath)
        qs.append(q)
        ts.append(t)
      print 'lbsS3', lbsS3
      print 'lbsR3', lbsR3
      idMax = np.argmax(lbsS3*lbsR3)
      print "choosing scale {} of run {}".format(cfg["lambdaS3"][idMax],idMax)
      q,t = qs[idMax], ts[idMax]
      lbS3, lbR3 = lbsS3[idMax], lbsR3[idMax]
      with open(transformationPath,'w') as f: # write best one back to file
        f.write("qw qx qy qz tx ty tz lbS3 lbR3\n")
        f.write("{} {} {} {} {} {} {} {} {}\n".format(
          q.q[0],q.q[1],q.q[2],q.q[3],t[0],t[1],t[2],lbS3,lbR3))

  if applyEGI:
    if loadCached and os.path.isfile(transformationPathEGI):
      print "found transformation file and using it " +transformationPathEGI
    else:
      args = ['matlab', '-nojvm', '-nodesktop', '-r',
      'compute_Rt_egi\(\''+scanApath+'\',\''+scanBpath+'\',\''+transformationPathEGI+'\'\)']
      print " ".join(args)
      err = subp.call(" ".join(args), shell=True)
    transformationPath = transformationPathEGI

  if applyMM:
    if loadCached and os.path.isfile(transformationPathMM):
      print "found transformation file and using it " +transformationPathMM
    else:
      args = ['../pod-build/bin/moment_matched_T3', 
          '-a {}'.format(scanApath), 
          '-b {}'.format(scanBpath), 
          '-o {}'.format(re.sub(".csv","",transformationPathMM))
          ]
      print " ".join(args)
      err = subp.call(" ".join(args), shell=True)
    transformationPath = transformationPathMM

  if applyICP:
    if loadCached and os.path.isfile(transformationPathICP):
      print "found transformation file and using it "+transformationPathICP
    else:
      args = ['../pod-build/bin/icp_T3', 
          '-a {}'.format(scanApath), 
          '-b {}'.format(scanBpath), 
          '-o {}'.format(re.sub(".csv","",transformationPathICP))
          ]
      if loadGlobalsolutionforICP:
        args.append('-t {}'.format(transformationPath))
      if useSurfaceNormalsInICP:
        args.append("-n")
      print " ".join(args)
      err = subp.call(" ".join(args), shell=True)
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
