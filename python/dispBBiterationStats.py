import numpy as np
import re
import matplotlib as mpl
import matplotlib.pyplot as plt
from js.utils.plot.colors import colorScheme

mpl.rc('font',size=35) 
mpl.rc('lines',linewidth=4.)
figSize = (14, 5.5)
figSize = (9, 12)
figSize = [(15.4, 10),  (15.4, 10), (15.4, 10), (12.6, 10)]

c1 = colorScheme("labelMap")["turquoise"]
c2 = colorScheme("labelMap")["orange"]
cs = [colorScheme("labelMap")["turquoise"],
    colorScheme("labelMap")["orange"],
    colorScheme("labelMap")["green"] ]


space = ["$Tp\mathbb{S}^{3}$", "AA", "$\mathbb{S}^{3}$", "$\mathbb{R}^3$"]
#space = ["$Tp\mathbb{S}^{3}$", "$\mathbb{S}^{3}$", "$\mathbb{R}^3$"]
disp = [1,1,1,0.7]

# volume of half of the sphere S^3 in 4D
VS3 = np.pi**2 

for i, path in enumerate(['./bb_iteration_stats_TpS3.csv', './bb_iteration_stats_AA.csv',
  './bb_iteration_stats_S3.csv', './bb_iteration_stats_R3.csv']):
  s = np.loadtxt(path).T
  s[:2,:] = np.log(s[:2,:])/np.log(10)
  Y = np.floor(s.shape[1]*disp[i])

  fig = plt.figure(figsize = figSize[i], dpi = 80, facecolor="w",
      edgecolor="k")
  ax1 = plt.subplot(2,1,1)
  
#  plt.plot(s[0,:Y],label="LB",color=c1)
#  plt.plot(s[1,:Y],label="convex UB",color=c2)
  plt.plot(s[1,:Y]-s[0,:Y],label="bound gap",color=c1)
  ax1.set_yscale("log", nonposy='clip')
#  if i == 0:
#    plt.legend(loc="best")
#  plt.xlabel("iterations of B&B")
  plt.ylabel("bound gap")
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
  print "saving result to {}".format(re.sub(".csv",".png",path))
#  plt.savefig(re.sub(".csv","_nodes.png",path), figure=fig)
#  plt.show()
  plt.close(fig)

paths = ['./bb_iteration_stats_TpS3.csv', './bb_iteration_stats_AA.csv', './bb_iteration_stats_S3.csv']
paths = ['./bb_iteration_stats_AA.csv', './bb_iteration_stats_S3.csv']
space = ["AA", "$\mathbb{S}^{3}$", "$\mathbb{R}^3$"]
cs = [#(0.1,0.1,0.1),
    colorScheme("labelMap")["red"],
    colorScheme("labelMap")["green"],
    colorScheme("labelMap")["green"] ]
Ys = []
fig = plt.figure(figsize = figSize[0], dpi = 80, facecolor="w", edgecolor="k")
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
  Ys.append(np.floor(s.shape[1]*disp[i]))
Y = np.max(np.array(Ys))

labelx = -0.09
ax1 = plt.subplot(211)
ax1.locator_params(axis="x", nbins=7, tight=True)
ax1.locator_params(axis="y", nbins=10, tight=True)
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
#  plt.plot(np.cumsum(s[4,1:Ys[i]]),s[1,1:Ys[i]]-s[0,1:Ys[i]],color=cs[i])
  plt.plot(np.arange(1,Ys[i]),s[1,1:Ys[i]]-s[0,1:Ys[i]],color=cs[i])
ax1.set_yscale("log", nonposy='clip')
ax1.grid(True)
plt.xlim([-Y/300., Y-1])
plt.ylabel("bound gap")
ax1.yaxis.set_label_coords(labelx, 0.5)
#plt.xlim([0, s[4,Y-1]])
plt.setp(ax1.get_xticklabels(), visible=False)
  
ax2 = plt.subplot(212, sharex=ax1)
#ax2.yaxis.get_major_formatter().set_powerlimits((0, 1))
ax2.ticklabel_format(axis='y', style='sci', scilimits=(0,1))
ax2.locator_params(axis="y", nbins=5, tight=True)
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
  plt.plot(s[2,:Ys[i]],label=space[i], color=cs[i])
#  plt.plot(np.cumsum(s[4,:Ys[i]]),s[2,:Ys[i]],label=space[i], color=cs[i])
ax2.set_ylabel("# nodes")
plt.xlim([-Y/300., Y-1])
ax2.yaxis.set_label_coords(labelx, 0.5)
ax2.grid(True)
plt.xlabel("iterations of BB")
#plt.legend(loc="best")
#plt.setp(ax2.get_xticklabels(), visible=False)
plt.savefig("./bb_iteration_stats_S3_bound_nodes.png", figure=fig)

fig2 = plt.figure(figsize = figSize[0], dpi = 80, facecolor="w", edgecolor="k")
ax3 = plt.subplot(211, sharex=ax1)
ax3.locator_params(axis="y", nbins=5, tight=True)
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
#  plt.plot(np.cumsum(s[4,:Ys[i]]),100.*s[3,:Ys[i]]/VS3, label=space[i], color=cs[i])
  plt.plot(100.*np.nan_to_num(s[3,:Ys[i]])/VS3, label=space[i], color=cs[i])
#  plt.plot(100.*(VS3-s[3,:Ys[i]])/VS3, label=space[i], color=cs[i])
plt.xlim([-Y/300., Y-1])
#ax3.set_yscale("log", nonposy='clip')
ax3.set_ylabel("% unexplored")
ax3.yaxis.set_label_coords(labelx, 0.5)
ax3.set_ylim([-1,150])
ax3.grid(True)
plt.setp(ax3.get_xticklabels(), visible=False)
plt.legend(loc="best")

ax4 = plt.subplot(212)
ax4.ticklabel_format(axis='y', style='sci', scilimits=(0,1))
ax4.locator_params(axis="y", nbins=5, tight=True)
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
  plt.plot(np.cumsum(s[4,1:Ys[i]])*1e-3,color=cs[i])
#  plt.plot(np.arange(1,Ys[i]),s[1,1:Ys[i]]-s[0,1:Ys[i]],color=cs[i])
#ax4.set_yscale("log", nonposy='clip')
ax4.grid(True)
plt.xlim([-Y/300., Y-1])
ax4.yaxis.set_label_coords(labelx, 0.5)
plt.ylabel("time [s]")
plt.xlabel("iterations of BB")
#plt.xlim([0, s[4,Y-1]])

plt.savefig("./bb_iteration_stats_S3_area_time.png", figure=fig2)
print "saving result to ./bb_iteration_stats_S3_bound_nodes.png and ./bb_iteration_stats_S3_area_time.png"


fig = plt.figure(figsize = figSize[0], dpi = 80, facecolor="w", edgecolor="k")
labelx = -0.09
ax1 = plt.subplot(211)
ax1.locator_params(axis="x", nbins=7, tight=True)
ax1.locator_params(axis="y", nbins=10, tight=True)
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
#  s[:2,:] = np.log(s[:2,:])/np.log(10)
  plt.plot(s[5,:Ys[i]],label=space[i],color=cs[i])
  plt.plot(s[6,:Ys[i]],color=cs[i])
#  plt.fill_between(np.arange(Ys[i]), s[0,:], s[1,:], color=cs[i])
ax1.set_yscale("log", nonposy='clip')
ax1.grid(True)
plt.ylabel("bounds of best node")
ax1.yaxis.set_label_coords(labelx, 0.5)
plt.xlim([0, Y-1])
plt.setp(ax1.get_xticklabels(), visible=False)

ax2 = plt.subplot(212)
ax2.locator_params(axis="x", nbins=7, tight=True)
ax2.locator_params(axis="y", nbins=10, tight=True)
for i, path in enumerate(paths):
  s = np.loadtxt(path).T
#  s[:2,:] = np.log(s[:2,:])/np.log(10)
  lvl = s[7,:Ys[i]]
  lvl[lvl>50] = 0
  plt.plot(lvl,color=cs[i])
#  plt.fill_between(np.arange(Ys[i]), s[0,:], s[1,:], color=cs[i])
ax2.grid(True)
plt.ylabel("depth of best node")
ax2.yaxis.set_label_coords(labelx, 0.5)
plt.xlim([0, Y-1])
plt.setp(ax2.get_xticklabels(), visible=False)

plt.show()
