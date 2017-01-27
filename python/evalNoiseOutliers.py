import numpy as np
from scipy.linalg import inv
from js.data.plyParse import PlyParse
import os.path, re, json, random, argparse
import subprocess as subp
from js.geometry.rotations import Quaternion
from js.utils.plot.colors import colorScheme
from helpers import *
import binascii

def SamplePair(inputPath, outlier, noiseStd, seedA, seedB, scale, outputPrefix):
  outputA = outputPrefix+"_A.ply";
  outputB = outputPrefix+"_B.ply";
  args = ['../build/bin/pclAddNoiseOutliers',
      '-i ' + inputPath,
      '-o ' + outputA,
      '-r {}'.format(outlier), 
      '-n {}'.format(noiseStd), 
      '-s {}'.format(scale), 
      '--seed {}'.format(int(seedA)), 
      ]
  print " ".join(args)
  validInput = (subp.call(" ".join(args), shell=True) == 0)
  args = ['../build/bin/pclAddNoiseOutliers',
      '-i ' + inputPath,
      '-o ' + outputB,
      '-r {}'.format(outlier), 
      '-n {}'.format(noiseStd), 
      '-s {}'.format(scale), 
      '--seed {}'.format(int(seedB)), 
      ]
  print " ".join(args)
  validInput = (subp.call(" ".join(args), shell=True) == 0)
  return outputA, outputB
def EvalError(q_gt, t_gt, q, t):
  if q is None or t is None:
    return np.nan, np.nan
  print "gt  rotation:    ", q_gt.q
  print "est rotation:    ", q.q
  print "gt  translation: ", t_gt
  print "est translation: ", t
  err_a = q_gt.angleTo(q)*180./np.pi
  err_t = np.sqrt(((t_gt-t)**2).sum())
  return err_a, err_t
def DisplayPcs(scanApath, scanBpath, q,t, plotCosies, stopToDisplay,
    displayNormals):
  from js.data.plyParse import PlyParse
  from js.utils.plot.colors import colorScheme
  from js.geometry.rotations import plotCosy
  import mayavi.mlab as mlab
  colors = colorScheme("label")
  print "parsing", scanApath
  plyA = PlyParse();
  plyA.parse(scanApath)
  pcA = plyA.getPc()
  nA = plyA.getNormals()
  print "parsing", scanBpath
  plyB = PlyParse();
  plyB.parse(scanBpath)
  pcB = plyB.getPc()
  nB = plyB.getNormals()

  R = q.toRot().R

  figm = mlab.figure(bgcolor=(1,1,1))
  mlab.points3d(pcA[:,0], pcA[:,1], pcA[:,2], mode="point",
      color=colors[0])
#  if plotCosies:
#    plotCosy(figm, np.eye(3), np.zeros(3), 0.5)
#    plotCosy(figm, R.T, -R.T.dot(t), 0.5)

  R = R.T
  t = -R.dot(t)
  pcB = (1.001*R.dot(pcB.T)).T + t
  nB = (1.001*R.dot(nB.T)).T

  mlab.points3d(pcB[:,0], pcB[:,1], pcB[:,2], mode="point",
        color=colors[1])

  if displayNormals:
    figm = mlab.figure(bgcolor=(1,1,1))
    mlab.points3d(nA[:,0], nA[:,1], nA[:,2], mode="point",
        color=colors[0])
    mlab.points3d(nB[:,0], nB[:,1], nB[:,2], mode="point",
          color=colors[1])

  if stopToDisplay:
    mlab.show(stop=True)

parser = argparse.ArgumentParser(description = 'randomly adds noise and outliers to pc')
parser.add_argument('-i','--input',
    default="../data/bunny/reconstruction/bun_zipper.ply", \
    help='path to input pointcloud .ply file or to a config file pointing to existing ply files s')
parser.add_argument('-o','--output',
    default="./", \
    help='path to output .json results file')
parser.add_argument('-p','--prefix',
    default=int(np.floor(time.time()*1e3)), \
    help='prefix for temp files')
parser.add_argument('-n','--noiseStd', default=0.0, help='noise std')
parser.add_argument('-r','--outlierRatio', default=0.0, help='outlier ratio')
parser.add_argument('-s','--scale', default=0.001, help='scale of PC')
parser.add_argument('-d','--display', action="store_true",
    help='display aligned point clouds')
cmdArgs = parser.parse_args()

runBB     =True
runBBICP  =True
showUntransformed = False
useSurfaceNormalsInICP = True

#cfg = {"name":"bunny", "lambdaS3": [60., 70., 80], "lambdaR3": 0.001,
#    "maxLvlR3":10, "maxLvlS3":11}
cfg = {"name":"bunny", "lambdaS3": [60.], "lambdaR3": 0.001,
    "maxLvlR3":10, "maxLvlS3":11}

version = "4.0" # initial

hash = cmdArgs.prefix
seedA = int(hash)%23752
seedB = int(hash)%91

scale = float(cmdArgs.scale)
noiseStd = float(cmdArgs.noiseStd)
outlier =float(cmdArgs.outlierRatio)
resultsPath = cmdArgs.output + "/" + os.path.splitext(os.path.split(cmdArgs.input)[1])[0]
outputPrefix = "./noise_{}_outlier_{}_{}".format(noiseStd,outlier,hash)
transformationPathBB = 'noise_{}_outlier_{}_{}_BB.csv'.format(noiseStd, outlier,hash)
transformationPathBBICP ='noise_{}_outlier_{}_{}_BB_ICP.csv'.format(noiseStd, outlier,hash)


q_gt = Quaternion(w=1., x=0., y=0., z=0.)
t_gt = np.zeros(3)

scanApath, scanBpath = SamplePair(cmdArgs.input, outlier, noiseStd,
    seedA, seedB, scale, outputPrefix)

if showUntransformed:
  q0 = Quaternion(1.,0.,0.,0.)
  DisplayPcs(scanApath, scanBpath, q0, np.zeros(3), True, True, False)

results = {"GT":{"q":q_gt.q.tolist(), "t":t_gt.tolist(),
  "overlap":1., "noiseStd": noiseStd, "outlier": outlier,
  # compute magnitude of translation as well as rotation of the two
  # viewpoints of the scene. 
  "dtranslation": 0,
  "dangle": 0 
  },
  "version":version}

if runBB:
  q,t,Ks,dt,success = RunBB(cfg, scanApath, scanBpath,
      transformationPathBB, simpleTranslation=False,
      tryMfAmbig=False)
  if not success:
    err_a, err_t = np.nan, np.nan
    if np.isnan(t).all(): # only translation is messed up -> err_a
      err_a, _ = EvalError(q_gt, t_gt, q, t)
    runBBICP = False
  else:
    err_a, err_t = EvalError(q_gt, t_gt, q, t)
  print "BB: {} deg {} m".format(err_a, err_t)
  results["BB"] = {"err_a":err_a, "err_t":err_t, "q":q.q.tolist(),
      "t":t.tolist(), "Ks":Ks.tolist(), "dt":dt}
  
if cmdArgs.display:
  DisplayPcs(scanApath, scanBpath, q,t, True, True, False)

if runBBICP:
  q,t,dt2,success = RunICP(scanApath, scanBpath, transformationPathBBICP,
      useSurfaceNormalsInICP, transformationPathBB)
  if not success:
    err_a, err_t = np.nan, np.nan
  else:
    err_a, err_t = EvalError(q_gt, t_gt, q, t)
  print "BB+ICP: {} deg {} m".format(err_a, err_t)
  results["BB+ICP"] = {"err_a":err_a, "err_t":err_t,
      "q":q.q.tolist(), "t":t.tolist(), "dt":dt+dt2}

if cmdArgs.display:
  DisplayPcs(scanApath, scanBpath, q,t, True, True, False)

import json, time
stamp = "{}".format(int(np.floor(time.time()*1e3)))
json.dump(results, open(resultsPath+"_"+stamp+'_results.json','w'))
print "done"
