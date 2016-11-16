import numpy as np
import os, re, time
import matplotlib.pyplot as plt

pattern = "apartment_[0-9]+resultsVsGrountruth.csv$"
paths = []
for algo in ["GoICP", "BB", "GOGMA", "FFT"]:
  for root, dirs, files in os.walk("./results/"+algo+"/"):
    for f in files:
      if re.search(pattern, f):
        paths.append(os.path.join(root, f))
    break

paths.sort()
paths = paths[::-1]

zone0 = np.array([2.5,0.5])
zone1 = np.array([5., 1.0])
zone2 = np.array([10.,2.0])

plt.figure()
for path in paths:
  print "--", time.ctime(os.path.getmtime(path))
  print path
  dRs = dict()
  dTs = dict()
  dts = dict()
  with open(path) as f:
    f.readline()
    print f.readline()[:-1]
    for line in f:
      vals = line[:-1].split(" ")
      dR = float(vals[3])
      dT = float(vals[4])
      dt = float(vals[5])
      if not vals[0] in dRs:
        dRs[vals[0]] = [dR]
      if not vals[0] in dTs:
        dTs[vals[0]] = [dT]
      if not vals[0] in dts:
        dts[vals[0]] = [dt]
      dRs[vals[0]].append(dR)
      dTs[vals[0]].append(dT)
      dts[vals[0]].append(dt)
  x = np.arange(45).astype(np.float)/44.
  label = ""
  for key,dR in dRs.iteritems():
    label += key
    cdf = np.sort(dR)
    plt.subplot(3,1,1)
    plt.plot(cdf, x[:cdf.size],label=label)
  for key,dt in dts.iteritems():
    cdf = np.sort(dt)
    plt.subplot(3,1,2)
    plt.plot(cdf, x[:cdf.size])
  for key,dT in dTs.iteritems():
    cdf = np.sort(dT)
    plt.subplot(3,1,3)
    plt.plot(cdf, x[:cdf.size])

plt.subplot(3,1,1)
plt.legend()
plt.xlabel("dR")
plt.ylim([0,1.01])

plt.subplot(3,1,2)
plt.xlabel("dt")
plt.ylim([0,1.01])

plt.subplot(3,1,3)
plt.ylim([0,1.01])
plt.xlabel("dT")

plt.show()

