import numpy as np
from scipy.linalg import solve, eig
import mayavi.mlab as mlab
from project4d import *
from helpers import *
from itertools import combinations, permutations

path = "./bb_nodes_per_iteration_S3_bunnyFull.csv"
qs, tetras, props = LoadNodesPerIteration(path)

show = False
q = np.array([0.707107, 0.381209, -0.0583624, 0.592683])
qp = q*1.4
q0 = q
n = normed(q0-qp)
cam = Project4d(qp, q0, n)

# plot
gold = (1.,215/255.,0)
silver = (192/255.,192/255.,192/255.)
scale = 0.03

propId = 2

def CreateLines(vs, tetras, props):
  # project
  vs3d = np.zeros((vs.shape[0],3))
  proppp = np.zeros(vs.shape[0])
  for i in range(vs.shape[0]):
    vs3d[i,:] = cam.Project(vs[i,:])
    ids = np.where(tetras == i)
    prop_i = np.zeros(ids[0].size)
    for j in range(ids[0].size):
      prop_i[j] = props[ids[0][j]][propId]
    proppp[i] = prop_i.min()
  xs , ys, zs, ss, edges = [], [], [], [], []
  n_points = 0
  for k, tetra in enumerate(tetras):
    for comb in combinations(range(4),2):
      i, j = tetra[comb[0]], tetra[comb[1]]
      ss.append(proppp[i])
      ss.append(proppp[j])
      xs.append(vs3d[i,0])
      xs.append(vs3d[j,0])
      ys.append(vs3d[i,1])
      ys.append(vs3d[j,1])
      zs.append(vs3d[i,2])
      zs.append(vs3d[j,2])
      edges.append([n_points*2, n_points*2+1])
      n_points += 1
  return xs, ys, zs, ss, edges

vmin = 99999.
vmax = -99999.
for prop in props:
  vmin = min(vmin, prop[:,propId].min())
  vmax = max(vmax, prop[:,propId].max())

xs, ys, zs, ss, edges = CreateLines(qs[0], tetras[0], props[0])
ranges = [np.array(xs).min(), np.array(xs).max(),
  np.array(ys).min(), np.array(ys).max(), np.array(zs).min(),
  np.array(zs).max()]

#for n in range(len(qs)):
figm = mlab.figure(bgcolor=(1,1,1))
for n in range(len(qs)):
  # Create the points
  xs, ys, zs, ss, edges = CreateLines(qs[n], tetras[n], props[n])
#  src = mlab.pipeline.scalar_scatter(xs, ys, zs, mode="sphere")
  # Connect them
  src = mlab.pipeline.scalar_scatter(np.array(xs), np.array(ys),
      np.array(zs), np.array(ss))
  pts = mlab.pipeline.glyph(src, scale_mode="none",
      opacity=1., scale_factor=scale*10, vmin=vmin, vmax=vmax) #, color=gold)
  src.mlab_source.dataset.lines = edges
  lines = mlab.pipeline.stripper(src)
  mlab.pipeline.surface(lines, color=silver, line_width=3.,
      opacity=.4)
  mlab.axes(extent=ranges, ranges=ranges)
  mlab.view(azimuth=2.*n/float(len(qs))*360, elevation=90.,distance=22.)
  mlab.savefig("frame_{:05}.png".format(n*2), figure=figm)
  # plot max
  idMax = np.argmax(props[n][:,1])
  xs, ys, zs = [], [], []
  for j in range(4):
    vs3d = cam.Project(qs[n][tetras[n][idMax, j],:])
    xs.append(vs3d[0])
    ys.append(vs3d[1])
    zs.append(vs3d[2])
  mlab.points3d(xs, ys, zs, scale_factor=11*scale, color=(1,0,0))
  mlab.axes(extent=ranges, ranges=ranges)
  mlab.view(azimuth=(2.*n+1)/float(len(qs))*360, elevation=90.,distance=22.)
  mlab.savefig("frame_{:05}.png".format(n*2+1), figure=figm)
  if show:
    mlab.show(stop=True)
  mlab.clf()
#  mlab.close(figm)

