import numpy as np
import os, re

pattern = "stairs_[0-9]+resultsVsGrountruth.csv$"
paths = []
for root, dirs, files in os.walk("./results/"):
  for f in files:
    if re.search(pattern, f):
      paths.append(os.path.join(root, f))

for path in paths:
  print path
  means = dict()
  with open(path) as f:
    f.readline()
    for line in f:
      vals = line[:-1].split(" ")
      val = np.array([float(vals[3]), float(vals[4]), float(vals[5]), 1.])
      if vals[0] in means:
        means[vals[0]] += val 
      else:
        means[vals[0]] = val
  for key,val in means.iteritems():
    print key, val/val[3]
