import numpy as np
from scipy.linalg import inv
from js.data.plyParse import PlyParse
import mayavi.mlab as mlab
import os.path, re
import subprocess as subp
from js.geometry.rotations import Quaternion
from js.utils.plot.colors import colorScheme
import argparse, sys
from helpers import *

def EvalError(q_gt, t_gt, q, t):
  if q is None or t is None:
    return np.nan, np.nan
  err_a = q_gt.angleTo(q)*180./np.pi
  err_t = np.sqrt(((t_gt-t)**2).sum())
  return err_a, err_t

cfgNYU = {"name":"nyu", "lambdaS3": [60, 70, 80], "lambdaR3": 1.}

cfg = cfgNYU

loadCached = True
stopToShow = True
showUntransformed = False
applyBB = True
applyEGI = False
applyMM = False
applyICP = True
loadGlobalsolutionforICP = True
useSurfaceNormalsInICP = True

#if cfg["name"] == "nyu":

parser = argparse.ArgumentParser(description = 'randomly sample two renders and align them')
parser.add_argument('-i','--input',
    default="../data/middle.ply", \
    help='path to input pointcloud .ply file')
parser.add_argument('-o','--output',
    default="./", \
    help='path to output .json results file')
parser.add_argument('-p','--prefix',
    default="out", \
    help='prefix for temp files')
parser.add_argument('-a','--angle', default=30., help='magnitude of random angle')
parser.add_argument('-t','--translation', default=1., help='magnitude of random translation')
parser.add_argument('-m','--minOverlap', default=70., help='min overlap between sampled point clouds')
cmdArgs = parser.parse_args()

resultsPath = cmdArgs.output + "/" + os.path.splitext(os.path.split(cmdArgs.input)[1])[0]
outputPath = "./" + cmdArgs.prefix
nameA = cmdArgs.prefix+"_A"
nameB = cmdArgs.prefix+"_B"
transformationPathBB = '{}_{}_BB.csv'.format(nameA, nameB)
transformationPathBBICP = '{}_{}_BB_ICP.csv'.format(nameA, nameB)
transformationPathICP = '{}_{}_ICP.csv'.format(nameA, nameB)
transformationPathEGI = '{}_{}_EGI.csv'.format(nameA, nameB)
transformationPathMM = '{}_{}_MM.csv'.format(nameA, nameB)
transformationPathMMICP = '{}_{}_MM_ICP.csv'.format(nameA, nameB)
transformationPathFFT = '{}_{}_FFT.csv'.format(nameA, nameB)
transformationPathFFTICP = '{}_{}_FFT_ICP.csv'.format(nameA, nameB)

runFFT = True
runFFTICP = True
runBB = True
runBBICP = True
runMM = True
runMMICP = True
runICP = True

args = ['../build/bin/renderPcFromPc',
    '-i ' + cmdArgs.input,
    '-o ' + outputPath,
    '-a {}'.format(cmdArgs.angle), 
    '-t {}'.format(cmdArgs.translation), 
    '-m {}'.format(cmdArgs.minOverlap), 
    ]
print " ".join(args)
if subp.call(" ".join(args), shell=True) == 0:
  paramString = 'angle_{}_translation_{}'.format(int(cmdArgs.angle),
      int(cmdArgs.translation))
  scanApath = outputPath+"_A_"+paramString+".ply"
  scanBpath = outputPath+"_B_"+paramString+".ply"
  # load gt
  gtPath = outputPath+"_"+paramString+"_TrueTransformation.csv"
  scanAtruePath = outputPath+"_"+paramString+"_Transformation_A_W.csv"
  scanBtruePath = outputPath+"_"+paramString+"_Transformation_B_W.csv"
  q_gt, t_gt, data_gt = LoadTransformationAndData(gtPath)
  q_A, t_A, data_A = LoadTransformationAndData(scanAtruePath)
  q_B, t_B, data_B = LoadTransformationAndData(scanBtruePath)
  R_A = q_A.toRot().R
  R_B = q_B.toRot().R
  R_BA = R_B.dot(R_A.T)
  t_BA = -R_BA.dot(t_A) + t_B

  results = {"GT":{"q":q_gt.q.tolist(), "t":t_gt.tolist(),
    "q_A_W":q_A.q.tolist(), "t_A_W":t_A.tolist(),
    "q_B_W":q_B.q.tolist(), "t_B_W":t_B.tolist(),
    "overlap":data_gt[0], "sizeA":data_gt[1], "sizeB": data_gt[2],
    "dtranslation": np.sqrt((t_BA**2).sum()),
    "dangle": q_A.angleTo(q_B)*180./np.pi
    },
    "version":"1.21"}

  if runFFT:
    q,t,success = RunFFT(scanApath, scanBpath, transformationPathFFT)
    if not success:
      err_a, err_t = np.nan, np.nan
      runFFTICP = False
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "FFT: {} deg {} m".format(err_a, err_t)
    results["FFT"] = {"err_a":err_a, "err_t":err_t, "q":q.q.tolist(),
        "t":t.tolist()}
    # write chosen transformation back to file for ICP
    if runFFTICP:
      with open(transformationPathFFT,'w') as f: 
        f.write("qw qx qy qz tx ty tz\n")
        f.write("{} {} {} {} {} {} {}\n".format(
          q.q[0],q.q[1],q.q[2],q.q[3],t[0],t[1],t[2]))

  if runFFTICP:
    q,t,success = RunICP(scanApath, scanBpath, transformationPathFFTICP,
        useSurfaceNormalsInICP, transformationPathFFT)
    if not success:
      err_a, err_t = np.nan, np.nan
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "FFT+ICP: {} deg {} m".format(err_a, err_t)
    results["FFT+ICP"] = {"err_a":err_a, "err_t":err_t,
        "q":q.q.tolist(), "t":t.tolist()}

  if runBB:
    q,t,success = RunBB(cfg, scanApath, scanBpath, transformationPathBB)
    if not success:
      err_a, err_t = np.nan, np.nan
      if np.isnan(t).all(): # only translation is messed up -> err_a
        err_a, _ = EvalError(q_gt, t_gt, q, t)
      runBBICP = False
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "BB: {} deg {} m".format(err_a, err_t)
    results["BB"] = {"err_a":err_a, "err_t":err_t, "q":q.q.tolist(),
        "t":t.tolist()}

  if runBBICP:
    q,t,success = RunICP(scanApath, scanBpath, transformationPathBBICP,
        useSurfaceNormalsInICP, transformationPathBB)
    if not success:
      err_a, err_t = np.nan, np.nan
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "BB+ICP: {} deg {} m".format(err_a, err_t)
    results["BB+ICP"] = {"err_a":err_a, "err_t":err_t,
        "q":q.q.tolist(), "t":t.tolist()}

  if runMM:
    q,t,success = RunMM(scanApath, scanBpath, transformationPathMM)
    if not success:
      err_a, err_t = np.nan, np.nan
      runMMICP = False
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "MM: {} deg {} m".format(err_a, err_t)
    results["MM"] = {"err_a":err_a, "err_t":err_t, "q":q.q.tolist(), "t":t.tolist()}

  if runMMICP:
    q,t,success = RunICP(scanApath, scanBpath, transformationPathMMICP,
        useSurfaceNormalsInICP, transformationPathMM)
    if not success:
      err_a, err_t = np.nan, np.nan
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "MM+ICP: {} deg {} m".format(err_a, err_t)
    results["MM+ICP"] = {"err_a":err_a, "err_t":err_t, "q":q.q.tolist(), "t":t.tolist()}

  if runICP:
    q,t,success = RunICP(scanApath, scanBpath, transformationPathICP,
        useSurfaceNormalsInICP)
    if not success:
      err_a, err_t = np.nan, np.nan
    else:
      err_a, err_t = EvalError(q_gt, t_gt, q, t)
    print "ICP: {} deg {} m".format(err_a, err_t)
    results["ICP"] = {"err_a":err_a, "err_t":err_t, "q":q.q.tolist(), "t":t.tolist()}

  import json, time
#  print results
#  import ipdb
#  ipdb.set_trace()
  stamp = "{}".format(int(np.floor(time.time()*1e3)))
  json.dump(results, open(resultsPath+"_"+stamp+'_results.json','w'))
  print "done"

  sys.exit(0)
sys.exit(1)
