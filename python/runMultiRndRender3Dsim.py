import subprocess as subp
import os.path
import re, random, argparse, time
import numpy as np
import time

# to generate different random prefixes even if all processes aree
# started at the same time with high probability
time.sleep(random.random()*5.)

parser = argparse.ArgumentParser(description = 'randomly sample two renders and align them')
parser.add_argument('-p','--prefix',
    default="{}".format(int(np.floor(time.time()*1e3))), \
    help='prefix for temp files')
cmdArgs = parser.parse_args()

prefix = cmdArgs.prefix

inPath = "/data/vision/fisher/expres1/jstraub/optRotTransCVPR2017_3Dsim/"
resultsPath = "/data/vision/fisher/expres1/jstraub/optRotTransCVPR2017/"
name = "config_[0-9]+.txt"

paths = []
for root, dirs, files in os.walk(inPath):
  for f in files:
    if re.search(name, f):
      paths.append(os.path.join(root,f))

print "found ", len(paths)
print "using prefix " + cmdArgs.prefix
random.shuffle(paths)

for path in paths:
  args=["python", "./randomRenderAndCompare.py", 
      "-i " + path,
      "-o " + resultsPath,
      "-p " + prefix]
  print " ".join(args)
  err = subp.call(" ".join(args), shell=True)
  if err == 1: # some problem with this scene?!
    print "aborting processing of "+f  
    break

#for data in datas:
#  for overlap in [50, 60, 70, 80, 90]:
#    args=["python", "./randomRenderAndCompare.py", 
#        "-i " + data,
#        "-m {}".format(overlap)]
#    err = subp.call(" ".join(args), shell=True)

