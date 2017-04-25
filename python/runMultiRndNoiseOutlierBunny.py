import subprocess as subp
import os.path
import re, random, argparse, time
import numpy as np
import time

# to generate different random prefixes even if all processes aree
# started at the same time with high probability
time.sleep(random.random()*5.)

parser = argparse.ArgumentParser(description = '')
parser.add_argument('-p','--prefix',
    default="{}".format(int(np.floor(time.time()*1e3))), \
    help='prefix for temp files')
cmdArgs = parser.parse_args()

prefix = cmdArgs.prefix

#nyuPath = "/media/jstraub/research/nyu_depth_v2/extracted/"
#resultsPath = "/media/jstraub/research/dpOptTrans/"

path = "../data/bunny/reconstruction/bun_zipper.ply"
resultsPath = "/data/vision/fisher/expres1/jstraub/optRotTransCVPR2017_bunny/"

print "using prefix " + cmdArgs.prefix
scale = 0.005 # 1/20 of bunny bounding radius

for it in range(1000):
  for outliers in [0.0, 0.3, 0.6]:
#    for noiseStd in [0.0, 0.002, 0.004, 0.006, 0.008, 0.01]:
#  for outliers in [0.0, 0.3, 0.6]:
#    for noiseStd in [0.001, 0.003]:
    for noiseStd in [0.0, 0.001, 0.002, 0.003, 0.004]:
      args=["python", "./evalNoiseOutliers.py", 
          "-i " + path,
          "-o " + resultsPath,
          "-p " + prefix,
          "-d ",
          "-r {}".format(outliers),
          "-s {}".format(scale),
          "-n {}".format(noiseStd)]
      print " ".join(args)
      err = subp.call(" ".join(args), shell=True)
      if err == 1: # some problem with this scene?!
        print "error processing of ", outliers, noiseStd

#noiseStd = 0.
#for outliers in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
#  args=["python", "./evalNoiseOutliers.py", 
#      "-i " + path,
#      "-o " + resultsPath,
#      "-p " + prefix,
#      "-r {}".format(outliers),
#      "-s {}".format(scale),
#      "-n {}".format(noiseStd)]
#  print " ".join(args)
#  err = subp.call(" ".join(args), shell=True)
#  if err == 1: # some problem with this scene?!
#    print "aborting processing of "+f  
#    break
#
#outliers = 0.
#for noiseStd in [0.0, 0.0001, 0.0005, 0.001, 0.002, 0.003, 0.004, 0.005, 0.01]:
#  args=["python", "./evalNoiseOutliers.py", 
#      "-i " + path,
#      "-o " + resultsPath,
#      "-p " + prefix,
#      "-r {}".format(outliers),
#      "-s {}".format(scale),
#      "-n {}".format(noiseStd)]
#  print " ".join(args)
#  err = subp.call(" ".join(args), shell=True)
#  if err == 1: # some problem with this scene?!
#    print "aborting processing of "+f  
#    break

