import numpy as np
from js.geometry.rotations import Quaternion
import re, os.path
import subprocess as subp

def RunFFT(scanApath, scanBpath, transformationPathFFT):
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
  q,t = LoadTransformation(transformationPathFFT)
  return q,t

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
    q,t,lbsS3[j],lbsR3[j] =\
      LoadTransformationAndBounds(transformationPathBB)
    qs.append(q)
    ts.append(t)
  print 'lbsS3', lbsS3
  print 'lbsR3', lbsR3
  idMax = np.argmax(lbsS3*lbsR3)
  print "choosing scale {} of run {}".format(cfg["lambdaS3"][idMax],idMax)
  q,t = qs[idMax], ts[idMax]
  lbS3, lbR3 = lbsS3[idMax], lbsR3[idMax]
  with open(transformationPathBB,'w') as f: # write best one back to file
    f.write("qw qx qy qz tx ty tz lbS3 lbR3\n")
    f.write("{} {} {} {} {} {} {} {} {}\n".format(
      q.q[0],q.q[1],q.q[2],q.q[3],t[0],t[1],t[2],lbS3,lbR3))
  return q,t
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
  q,t = LoadTransformation(transformationPathMM)
  return q,t
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
  q,t = LoadTransformation(transformationPathICP)
  return q,t

def LoadTransformation(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    if len(qt.shape) == 1:
      q = qt[:4]
      t = qt[4:7]
    else:
      q = qt[0,:4]
      t = qt[0,4:7]
    print 'q', q
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
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

def LoadTransformationAndOverlap(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    if len(qt.shape) == 1:
      q = qt[:4]
      t = qt[4:7]
      overlap = qt[7]
    else:
      q = qt[0,:4]
      t = qt[0,4:7]
      overlap = qt[0,7]
    print 'q', q
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
  return q,t, overlap
