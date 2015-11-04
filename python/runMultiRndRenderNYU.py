import subprocess as subp
import os.path
import re, random, argparse, time
import numpy as np

parser = argparse.ArgumentParser(description = 'randomly sample two renders and align them')
parser.add_argument('-p','--prefix',
    default="{}".format(int(np.floor(time.time()*1e3))), \
    help='prefix for temp files')
cmdArgs = parser.parse_args()

prefix = cmdArgs.prefix

nyuPath = "/data/vision/fisher/data1/nyu_depth_v2/extracted/"
resultsPath = "/data/vision/fisher/expres1/jstraub/optRotTrans/"
name = "[a-z_]+_[0-9]+_[0-9]+.ply"

paths = []
for root, dirs, files in os.walk(nyuPath):
  for f in files:
    if re.search(name, f):
      paths.append(os.path.join(root,f))

print "found ", len(paths)
print "using prefix " + cmdArgs.prefix
random.shuffle(paths)

for path in paths:
  for overlap in [30, 40, 50, 60, 70, 80, 90]:
    args=["python", "./randomRenderAndCompare.py", 
        "-i " + path,
        "-o " + resultsPath,
        "-p " + prefix,
        "-m {}".format(overlap)]
    print " ".join(args)
    err = subp.call(" ".join(args), shell=True)
    if err == 1: # some problem with this scene?!
      print "aborting processing of "+f  
      break
