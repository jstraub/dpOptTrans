import numpy as np
import os.path, re, json

from helpers import *

def logDeviations(fRes,pathGtA,pathGtB,q_ba,t_ba,dt,algo):
  # loaded transformation is T_wc
  q_gtA,t_gtA,_ = LoadTransformation(pathGtA)
  q_gtB,t_gtB,_ = LoadTransformation(pathGtB)
  dq_gt = q_gtB.inverse().dot(q_gtA)
  dt_gt = q_gtB.inverse().rotate(t_gtA - t_gtB)
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

pattern = "HokuyoPcNormals_[0-9]+_HokuyoPcNormals_[0-9]+_Gogma.txt"
resFiles = []
for root, dirs, files in os.walk("./"):
  for f in files:
    if re.search(pattern, f):
      resFiles.append(os.path.join(root, f))
resFiles = sorted(resFiles, key=lambda f: 
  int(re.sub("_HokuyoPcNormals_[0-9]+_Gogma.txt","",re.sub("^HokuyoPcNormals_","",os.path.split(f)[1]))))

pattern = "pose_[0-9]+.csv$"
gt = []
for root, dirs, files in os.walk("../data/apartment/"):
  for f in files:
    if re.search(pattern, f):
      gt.append(os.path.join(root, f))
gt = sorted(gt, key=lambda f: 
  int(re.sub(".csv","",
    re.sub("pose_","",os.path.split(f)[1]))))

prefix = "apartment_{}".format(int(np.floor(time.time()*1e3)))
fRes = open(prefix+"resultsVsGrountruth.csv","w")
fRes.write("algo idFrom idTo dAngDeg dTrans dTimeSec\n")
fRes.write(" {} \n")
fRes.flush()

i = 1
for resF in resFiles:
  q_ba, t_ba, q_baGlobal, t_baGlobal = LoadTransformationGogma(resF)
  print resF
  print gt[i-1]
  print gt[i]
  logDeviations(fRes, gt[i-1], gt[i], q_baGlobal,t_baGlobal,0.,"GOGMAonly")
  i+=1
fRes.close()
