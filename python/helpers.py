import numpy as np
from js.geometry.rotations import Quaternion

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

def LoadTransformationAndOverlap(transformationPath):
  with open(transformationPath) as f:
    f.readline()
    qt = np.loadtxt(f)
    q = qt[:4];
    print 'q', q
    t = qt[4:7];
    print 't', t
    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    overlap = qt[7]
  return q,t, overlap
