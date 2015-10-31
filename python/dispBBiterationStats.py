import numpy as np
import re
import matplotlib as mpl
import matplotlib.pyplot as plt
from js.utils.plot.colors import colorScheme

mpl.rc('font',size=25) 
mpl.rc('lines',linewidth=3.)
figSize = (14, 5.5)
figSize = (14, 10)
figSize = (14, 12)

c1 = colorScheme("labelMap")["turquoise"]
c2 = colorScheme("labelMap")["orange"]

space = ["$\mathbb{S}^{3}$", "$\mathbb{R}^3$"]
disp = [1,0.8]

for i, path in enumerate(['./bb_iteration_stats_S3.csv',
  './bb_iteration_stats_R3.csv']):
  s = np.loadtxt(path).T
  s[:2,:] = np.log(s[:2,:])/np.log(10)
  Y = np.floor(s.shape[1]*disp[i])

  fig = plt.figure(figsize = figSize, dpi = 80, facecolor="w",
      edgecolor="k")
  ax = plt.subplot(2,1,1)
#  ax.set_yscale("log")
  
  plt.plot(s[0,:Y],label="lower bound",color=c1)
  plt.plot(s[1,:Y],label="joint upper bound",color=c2)
  plt.legend(loc="best")
  plt.xlabel("iterations of B&B")
  plt.ylabel("log$_{10}$(bounds)")
  plt.ylim([s[:2,:].min(), s[:2,:].max()])
  plt.xlim([0, Y-1])
  
  ax = plt.subplot(2,1,2)
  plt.plot(s[2,:Y],label="number of nodes", color=c1)
  plt.xlabel("iterations of B&B")
  ax.set_ylabel("# nodes in B&B", color=c1)
  for tl in ax.get_yticklabels():
    tl.set_color(c1)
  ax2 = ax.twinx()
  plt.plot(100.*(s[3,0]-s[3,:Y])/s[3,0], color=c2)
  ax2.set_ylabel("% of "+space[i]+" explored", color=c2)
  ax2.set_ylim([0,101])
  plt.xlim([0, Y-1])
  for tl in ax2.get_yticklabels():
    tl.set_color(c2)
  plt.savefig(re.sub("csv","png",path), figure=fig)
  plt.show()

