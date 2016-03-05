import numpy as np
import re
import matplotlib as mpl
import matplotlib.pyplot as plt
from js.utils.plot.colors import colorScheme

mpl.rc('font',size=35) 
mpl.rc('lines',linewidth=4.)
figSize = (14, 5.5)
figSize = (9, 12)
figSize = [(15.4, 10), (15.4, 10), (12.6, 10)]

c1 = colorScheme("labelMap")["turquoise"]
c2 = colorScheme("labelMap")["orange"]

space = ["$Tp\mathbb{S}^{3}$", "$\mathbb{S}^{3}$", "$\mathbb{R}^3$"]
disp = [1,1,0.7]

for i, path in enumerate(['./bb_iteration_stats_TpS3.csv','./bb_iteration_stats_S3.csv',
  './bb_iteration_stats_R3.csv']):
  s = np.loadtxt(path).T
  s[:2,:] = np.log(s[:2,:])/np.log(10)
  Y = np.floor(s.shape[1]*disp[i])

  fig = plt.figure(figsize = figSize[i], dpi = 80, facecolor="w",
      edgecolor="k")
  ax1 = plt.subplot(2,1,1)
  
  plt.plot(s[0,:Y],label="LB",color=c1)
  plt.plot(s[1,:Y],label="convex UB",color=c2)
  if i == 0:
    plt.legend(loc="best")
#  plt.xlabel("iterations of B&B")
  plt.ylabel("log$_{10}$(bound)")
#  plt.ylim([s[:2,:].min(), s[:2,:].max()])
  plt.xlim([0, Y-1])
  plt.tight_layout()
  plt.setp(ax1.get_xticklabels(), visible=False)
#  plt.savefig(re.sub(".csv","_bounds.png",path), figure=fig)
  
#  fig = plt.figure(figsize = figSize, dpi = 80, facecolor="w",
#      edgecolor="k")
  ax2 = plt.subplot(212, sharex=ax1)
  plt.plot(s[2,:Y],label="number of nodes", color=c1)
  plt.xlabel("iterations of BB")
  ax2.set_ylabel("# nodes in BB", color=c1)
  for tl in ax2.get_yticklabels():
    tl.set_color(c1)
  ax2.set_yticks(ax2.get_yticks()[:-1])
  ax3 = ax2.twinx()
  plt.plot(100.*(s[3,0]-s[3,:Y])/s[3,0], color=c2)
  ax3.set_ylabel("% of "+space[i]+" explored", color=c2)
  ax3.set_ylim([0,101])
  plt.xlim([0, Y-1])
  for tl in ax3.get_yticklabels():
    tl.set_color(c2)
  ax3.set_yticks(ax3.get_yticks()[:-1])
  ax3.set_xticks(ax3.get_xticks()[:-1])
  plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.1)
  plt.savefig(re.sub(".csv",".png",path), figure=fig)
#  plt.savefig(re.sub(".csv","_nodes.png",path), figure=fig)
#  plt.show()

