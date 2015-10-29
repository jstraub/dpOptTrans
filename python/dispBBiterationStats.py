import numpy as np
import re
import matplotlib as mpl
import matplotlib.pyplot as plt

mpl.rc('font',size=25) 
mpl.rc('lines',linewidth=3.)
figSize = (14, 5.5)
figSize = (14, 10)
figSize = (14, 12)

for path in ['./bb_iteration_stats_S3.csv', './bb_iteration_stats_R3.csv']:
  s = np.loadtxt(path).T
  s[:2,:] = np.log(s[:2,:])/np.log(10)

  fig = plt.figure(figsize = figSize, dpi = 80, facecolor="w",
      edgecolor="k")
  ax = plt.subplot(2,1,1)
#  ax.set_yscale("log")
  plt.plot(s[0,:],label="lower bound")
  plt.plot(s[1,:],label="joint convex upper bound")
  plt.legend()
  plt.xlabel("iterations of B&B")
  plt.ylabel("log$_{10}$(bounds)")
  plt.ylim([s[:2,:].min(), s[:2,:].max()])
  plt.subplot(2,1,2)
  plt.plot(s[2,:],label="number of nodes")
  plt.xlabel("iterations of B&B")
  plt.ylabel("# nodes in B&B")
  plt.savefig(re.sub("csv","png",path), figure=fig)
  plt.show()

