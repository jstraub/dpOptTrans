import numpy as np
from js.geometry.rotations import Quaternion
import re, os.path
import subprocess as subp

def RunFFT(scanApath, scanBpath, transformationPathFFT, q_gt=None,
    t_gt=None, returnBoth=False):
  scanApathAbs = os.path.abspath(scanApath)
  scanBpathAbs = os.path.abspath(scanBpath)
  transformationPathFFTAbs = os.path.abspath(transformationPathFFT)
  args = ["./runMatlabFFT.sh", 
      '\'' + scanApathAbs + '\'',
      '\'' + scanBpathAbs + '\'',
      '\'' + transformationPathFFTAbs+'\'']
  print " ".join(args)
  err = subp.call(" ".join(args), shell=True)
  if err > 0:
    print "ERROR in run FFT"
    return Quaternion(), np.zeros(3), False
  qs,ts = LoadTransformation(transformationPathFFT)
  id_best = 0
  if not (q_gt is None and t_gt is None):
    dist = np.zeros(len(qs))
    for i in range(len(qs)):
      dist[i] = q_gt.angleTo(qs[i])
    print dist
    id_best = np.argmin(dist)
  if returnBoth:
    return qs[id_best], ts[id_best], qs[0], ts[0], True
  else:
    return qs[id_best], ts[id_best], True

def RunBB(cfg, scanApath, scanBpath, transformationPathBB):
  lbsS3 = np.zeros(len(cfg["lambdaS3"]))
  lbsR3 = np.zeros(len(cfg["lambdaS3"]))
  qs, ts = [],[]
  for j,lambdaS3 in enumerate(cfg["lambdaS3"]):
    args = ['../pod-build/bin/dpvMFoptRotPly', 
        '-a {}'.format(scanApath), 
        '-b {}'.format(scanBpath), 
        '-l {}'.format(lambdaS3),
        '-t {}'.format(cfg["lambdaR3"]),
        '-o ' + re.sub(".csv","",transformationPathBB)
        ]
    print " ".join(args)
    err = subp.call(" ".join(args), shell=True)
    if err > 0:
      print "ERROR in RunBB"
      return Quaternion(), np.zeros(3), False
    q,t,lbsS3[j],lbsR3[j] =\
      LoadTransformationAndBounds(transformationPathBB)
    qs.append(q)
    ts.append(t)
  print 'lbsS3', lbsS3
  print 'lbsR3', lbsR3
  if np.logical_or(np.isinf(lbsR3), np.isnan(lbsR3)).all():
    return q, np.array([np.nan, np.nan, np.nan]), False
  idMax = np.argmax(lbsS3*lbsR3)
  print "choosing scale {} of run {}".format(cfg["lambdaS3"][idMax],idMax)
  q,t = qs[idMax], ts[idMax]
  lbS3, lbR3 = lbsS3[idMax], lbsR3[idMax]
  with open(transformationPathBB,'w') as f: # write best one back to file
    f.write("qw qx qy qz tx ty tz lbS3 lbR3\n")
    f.write("{} {} {} {} {} {} {} {} {}\n".format(
      q.q[0],q.q[1],q.q[2],q.q[3],t[0],t[1],t[2],lbS3,lbR3))
  return q,t, True
def RunMM(scanApath, scanBpath, transformationPathMM):
  args = ['../pod-build/bin/moment_matched_T3', 
      '-a {}'.format(scanApath), 
      '-b {}'.format(scanBpath), 
      '-o {}'.format(re.sub(".csv","",transformationPathMM))
      ]
  print " ".join(args)
  err = subp.call(" ".join(args), shell=True)
  if err > 0:
    print "ERROR in run MM"
    return Quaternion(), np.zeros(3), False
  q,t = LoadTransformation(transformationPathMM)
  return q,t, True
def RunICP(scanApath, scanBpath, transformationPathICP,
    useNormals=True, transformationPathGlobal=None):
  args = ['../pod-build/bin/icp_T3', 
      '-a {}'.format(scanApath), 
      '-b {}'.format(scanBpath), 
      '-o {}'.format(re.sub(".csv","",transformationPathICP))
      ]
  if not transformationPathGlobal is None:
    args.append('-t {}'.format(transformationPathGlobal))
  if useNormals:
    args.append("-n")
  print " ".join(args)
  err = subp.call(" ".join(args), shell=True)
  if err > 0:
    print "ERROR in run ICP"
    return Quaternion(), np.zeros(3), False
  q,t = LoadTransformation(transformationPathICP)
  return q,t,True

def LoadTransformation(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    if len(qt.shape) == 1:
      q = Quaternion(w=qt[0], x=qt[1], y=qt[2], z=qt[3])
      t = qt[4:7]
    else:
      q = []
      t = []
      for i in range(qt.shape[0]):
        q.append(Quaternion(w=qt[i,0], x=qt[i,1], y=qt[i,2], z=qt[i,3]))
        t.append(qt[i,4:7])
    print 'q', q
    print 't', t
  return q,t

def LoadTransformationAndBounds(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    if len(qt.shape) == 1:
      q = qt[:4]
      t = qt[4:7]
      lbS3 = qt[7]
      lbR3 = qt[8]
    else:
      q = qt[0,:4]
      t = qt[0,4:7]
      lbS3 = qt[0,7]
      lbR3 = qt[0,8]
    print 'q', q
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
  return q,t, lbS3, lbR3

def LoadTransformationAndData(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    if len(qt.shape) == 1:
      q = qt[:4]
      t = qt[4:7]
      data = qt[7:]
    else:
      q = qt[0,:4]
      t = qt[0,4:7]
      data = qt[0,:]
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
  return q,t,data
