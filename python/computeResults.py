import numpy as np
import os, re, time

pattern = "apartment_[0-9]+resultsVsGrountruth.csv$"
paths = []
for algo in ["GoICP", "BB", "GOGMA", "FFT"]:
  for root, dirs, files in os.walk("./results/"+algo+"/"):
    for f in files:
      if re.search(pattern, f):
        paths.append(os.path.join(root, f))

paths.sort()
paths = paths[::-1]

zone0 = np.array([2.5,0.5])
zone1 = np.array([5., 1.0])
zone2 = np.array([10.,2.0])

for path in paths:
  print "--", time.ctime(os.path.getmtime(path))
  print path
  means = dict()
  stats = dict()
  with open(path) as f:
    f.readline()
    print f.readline()[:-1]
    for line in f:
      vals = line[:-1].split(" ")
      val = np.array([float(vals[3]), float(vals[4]), float(vals[5]), 1.])
      if not vals[0] in means:
        means[vals[0]] = np.zeros(4)
        stats[vals[0]] = np.zeros(3)
      means[vals[0]] += val 
      if np.all(val[:2] < zone0):
        stats[vals[0]][2] += 1
      if np.all(val[:2] < zone1):
        stats[vals[0]][1] += 1
      if np.all(val[:2] < zone2):
        stats[vals[0]][0] += 1
  for key,val in means.iteritems():
    print key, val/val[3], val[3]
    print key, 100.*stats[key]/val[3]
    for x in val[:2]/val[3]:
      print "{: >6.2f}".format(x)
    for x in 100.*stats[key]/val[3]:
      print "{: >6.2f}".format(x)
    print "{: >6.2f}".format(val[2]/val[3])
