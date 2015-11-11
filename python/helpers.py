import numpy as np
from js.geometry.quaternion import Quaternion
import re, os.path, time
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
  t0 = time.time()
  err = subp.call(" ".join(args), shell=True)
  dt = time.time() - t0
  print "dt: ", dt, "[s]"
  if err > 0:
    print "ERROR in run FFT"
    return Quaternion(), np.zeros(3), dt, False
  qs,ts = LoadTransformation(transformationPathFFT)
  id_best = 0
  if not (q_gt is None and t_gt is None):
    dist = np.zeros(len(qs))
    for i in range(len(qs)):
      dist[i] = q_gt.angleTo(qs[i])
    print dist
    id_best = np.argmin(dist)
  if returnBoth:
    return qs[id_best], ts[id_best], qs[0], ts[0], dt, True
  else:
    return qs[id_best], ts[id_best], dt, True
def RunBBsimple(scanApath, scanBpath, transformationPathBB, lambdaS3,
    lambdaR3, EGImode=False):
  args = ['../pod-build/bin/dpvMFoptRotPly', 
      '-a {}'.format(scanApath), 
      '-b {}'.format(scanBpath), 
      '-l {}'.format(lambdaS3),
      '-t {}'.format(lambdaR3),
      '-o ' + re.sub(".csv","",transformationPathBB)
      ]
  if EGImode:
    args.append('-e')
  print " ".join(args)
  t0 = time.time()
  err = subp.call(" ".join(args), shell=True)
  dt = time.time() - t0
  if err > 0:
    print "ERROR in RunBB"
    return Quaternion(), np.zeros(3), np.zeros(4), dt, False
  q,t,lbS3,lbR3, KvmfA, KvmfB, KgmmA, KgmmB =\
    LoadTransformationAndBounds(transformationPathBB)
  Ks = np.array([KvmfA, KvmfB, KgmmA, KgmmB])
  print "dt: ", dt, "[s]"
  if np.logical_or(np.isinf([lbR3]), np.isnan([lbR3])).all():
    return q, np.array([np.nan, np.nan, np.nan]), Ks,dt,False
  return q,t, Ks, dt,True
def RunBB(cfg, scanApath, scanBpath, transformationPathBB,\
    EGImode=False):
  lbsS3 = np.zeros(len(cfg["lambdaS3"]))
  lbsR3 = np.zeros(len(cfg["lambdaS3"]))
  Ks = np.zeros((len(cfg["lambdaS3"]), 4))
  qs, ts = [],[]
  dt = 0.
  for j,lambdaS3 in enumerate(cfg["lambdaS3"]):
    args = ['../pod-build/bin/dpvMFoptRotPly', 
        '-a {}'.format(scanApath), 
        '-b {}'.format(scanBpath), 
        '-l {}'.format(lambdaS3),
        '-t {}'.format(cfg["lambdaR3"]),
        '-o ' + re.sub(".csv","",transformationPathBB)
        ]
    if EGImode:
      args.append('-e')
    print " ".join(args)
    t0 = time.time()
    err = subp.call(" ".join(args), shell=True)
    dt += time.time() - t0
    if err > 0:
      print "ERROR in RunBB"
      return Quaternion(), np.zeros(3), np.zeros(4), dt, False
    q,t,lbsS3[j],lbsR3[j], KvmfA, KvmfB, KgmmA, KgmmB =\
      LoadTransformationAndBounds(transformationPathBB)
    qs.append(q)
    ts.append(t)
    Ks[j,0], Ks[j,1], Ks[j,2], Ks[j,3] = KvmfA, KvmfB, KgmmA, KgmmB
  print 'lbsS3', lbsS3
  print 'lbsR3', lbsR3
  print "dt: ", dt, "[s]"
  if np.logical_or(np.isinf(lbsR3), np.isnan(lbsR3)).all():
    idMax = np.argmax(lbsS3)
    return qs[idMax], np.array([np.nan, np.nan, np.nan]), Ks[idMax,:],dt,False
  idMax = np.argmax(lbsS3*lbsR3)
  print "choosing scale {} of run {}".format(cfg["lambdaS3"][idMax],idMax)
  q,t = qs[idMax], ts[idMax]
  lbS3, lbR3 = lbsS3[idMax], lbsR3[idMax]
  KvmfA, KvmfB, KgmmA, KgmmB = Ks[idMax,0], Ks[idMax,1], Ks[idMax,2], Ks[idMax,3]
  with open(transformationPathBB,'w') as f: # write best one back to file
    f.write("qw qx qy qz tx ty tz lbS3 lbR3 KvmfA KvmfB KgmmA KgmmB\n")
    f.write("{} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(
      q.q[0],q.q[1],q.q[2],q.q[3],t[0],t[1],t[2],lbS3,lbR3,KvmfA,\
      KvmfB, KgmmA, KgmmB))
  return q,t, Ks[idMax,:], dt,True
def RunMM(scanApath, scanBpath, transformationPathMM):
  args = ['../pod-build/bin/moment_matched_T3', 
      '-a {}'.format(scanApath), 
      '-b {}'.format(scanBpath), 
      '-o {}'.format(re.sub(".csv","",transformationPathMM))
      ]
  print " ".join(args)
  t0 = time.time()
  err = subp.call(" ".join(args), shell=True)
  dt = time.time() - t0
  print "dt: ", dt, "[s]"
  if err > 0:
    print "ERROR in run MM"
    return Quaternion(), np.zeros(3), dt, False
  q,t = LoadTransformation(transformationPathMM)
  return q,t, dt,True
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
  t0 = time.time()
  err = subp.call(" ".join(args), shell=True)
  dt = time.time() - t0
  print "dt: ", dt, "[s]"
  if err > 0:
    print "ERROR in run ICP"
    return Quaternion(), np.zeros(3), dt,False
  q,t = LoadTransformation(transformationPathICP)
  return q,t,dt,True

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
      KvmfA, KvmfB, KgmmA, KgmmB = qt[9],qt[10],qt[11],qt[12]
    else:
      q = qt[0,:4]
      t = qt[0,4:7]
      lbS3 = qt[0,7]
      lbR3 = qt[0,8]
      KvmfA, KvmfB, KgmmA, KgmmB = qt[0,9],qt[0,10],qt[0,11],qt[0,12]
    print 'q', q
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
  return q,t, lbS3, lbR3, KvmfA, KvmfB, KgmmA, KgmmB

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

def LoadNodesPerIteration(path, maxNodes=None):
  if re.search("S3", path):
    with open(path, 'r') as f:
      qs, tetras, props = [], [], []
      while f.read(1):
        f.seek(-1,1)
        stats = np.fromstring(f.readline(), sep=" ")
        print stats
        N = int(stats[2])
        qs.append(np.zeros((N*4,4)))
        tetras.append(np.zeros((N,4), dtype=np.int))
        props.append(np.zeros((N,4)))
        j,k = 0,0
        for i in range(N*5):
          if i%5 == 0:
            props[-1][i/5,:] = np.fromstring(f.readline(), sep=" ")
            tetras[-1][i/5,:] = [j, j+1, j+2, j+3]
            k = 0
          else:
            q = np.fromstring(f.readline(), sep=" ")
            if j==0:
              qs[-1][j,:] = q
              j+=1
            else:
              thetas = np.arccos(np.minimum(1.,np.maximum(-1.,q.dot(qs[-1][:j,:].T))))
              if np.any(thetas < 1.e-6):
                tetras[-1][i/5,k] = np.where(thetas < 1e-6)[0][0]
              else:
                qs[-1][j,:] = q
                tetras[-1][i/5,k] = j
                j+=1
            k+=1
        qs[-1] = np.resize(qs[-1], (j,4))
        if not maxNodes is None and len(qs) >= maxNodes:
          break
    return qs, tetras, props
  if re.search("R3", path):
    with open(path, 'r') as f:
      ts, tetras, props = [], [], []
      while f.read(1):
        f.seek(-1,1)
        stats = np.fromstring(f.readline(), sep=" ")
        print stats
        N = int(stats[2])
        ts.append(np.zeros((N*4,3)))
        tetras.append(np.zeros((N,8), dtype=np.int))
        props.append(np.zeros((N,4)))
        j,k = 0,0
        for i in range(N*9):
          if i%9 == 0:
            props[-1][i/9,:] = np.fromstring(f.readline(), sep=" ")
#            tetras[-1][i/9,:] = [j, j+1, j+2, j+3]
            k = 0
          else:
            t = np.fromstring(f.readline(), sep=" ")
            if j==0:
              ts[-1][j,:] = t
              j+=1
            else:
              thetas = np.sqrt(((ts[-1][:j,:]-t)**2).sum(axis=1))
#              print "t1++ "
#              print thetas
              if np.any(thetas < 1.e-6):
                tetras[-1][i/9,k] = np.where(thetas < 1e-6)[0][0]
              else:
                ts[-1][j,:] = t
                tetras[-1][i/9,k] = j
                j+=1
            k+=1
        ts[-1] = np.resize(ts[-1], (j,3))
        print ts[-1].shape
        if not maxNodes is None and len(ts) >= maxNodes:
          break
    return ts, tetras, props

