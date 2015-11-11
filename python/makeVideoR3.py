import numpy as np
from scipy.linalg import solve, eig
import mayavi.mlab as mlab
from project4d import *
from helpers import *
from itertools import combinations, permutations

path = "./bb_nodes_per_iteration_R3_bunnyFull.csv"
ts, tetras, props = LoadNodesPerIteration(path)

show = False
q = np.array([0.707107, 0.381209, -0.0583624, 0.592683])

# plot
gold = (1.,215/255.,0)
silver = (192/255.,192/255.,192/255.)
scale = 0.001

propId = 2

def CreateLines(vs, tetras, props):
  # project
  proppp = np.zeros(vs.shape[0])
  for i in range(vs.shape[0]):
    ids = np.where(tetras == i)
    prop_i = np.zeros(ids[0].size)
    for j in range(ids[0].size):
      prop_i[j] = props[ids[0][j]][propId]
    proppp[i] = prop_i.min()
  xs , ys, zs, ss, edges = [], [], [], [], []
  n_points = 0
  for k, tetra in enumerate(tetras):
    for comb in combinations(range(8),2):
      i, j = tetra[comb[0]], tetra[comb[1]]
      if (np.abs(vs[j,:] - vs[i,:]) < 1e-6).sum() == 2:
        ss.append(proppp[i])
        ss.append(proppp[j])
        xs.append(vs[i,0])
        xs.append(vs[j,0])
        ys.append(vs[i,1])
        ys.append(vs[j,1])
        zs.append(vs[i,2])
        zs.append(vs[j,2])
        edges.append([n_points*2, n_points*2+1])
        n_points += 1
  return xs, ys, zs, ss, edges

vmin = 99999.
vmax = -99999.
for prop in props:
  vmin = min(vmin, prop[:,propId].min())
  vmax = max(vmax, prop[:,propId].max())

xs, ys, zs, ss, edges = CreateLines(ts[0], tetras[0], props[0])
ranges = [np.array(xs).min(), np.array(xs).max(),
  np.array(ys).min(), np.array(ys).max(), np.array(zs).min(),
  np.array(zs).max()]

#for n in range(len(ts)):
figm = mlab.figure(bgcolor=(1,1,1))
for n in range(len(ts)):
  # Create the points
  xs, ys, zs, ss, edges = CreateLines(ts[n], tetras[n], props[n])
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
  mlab.view(azimuth=45+2.*n/float(len(ts))*360,
      elevation=54.,distance=1.5, 
      focalpoint=np.array([0.0970635 , -0.22400211,  0.036377]))
  print mlab.view()
  mlab.savefig("frame_R3_{:05}.png".format(n*2), figure=figm)
  # plot max
  idMax = np.argmax(props[n][:,1])
  xs, ys, zs = [], [], []
  for j in range(8):
    vs = ts[n][tetras[n][idMax, j],:]
    xs.append(vs[0])
    ys.append(vs[1])
    zs.append(vs[2])
  mlab.points3d(xs, ys, zs, scale_factor=11*scale, color=(1,0,0))
  mlab.axes(extent=ranges, ranges=ranges)
  mlab.view(azimuth=45+2.*n/float(len(ts))*360,
      elevation=54.,distance=1.5, 
      focalpoint=np.array([0.0970635 , -0.22400211,  0.036377]))
  mlab.savefig("frame_R3_{:05}.png".format(n*2+1), figure=figm)
  if show:
    mlab.show(stop=True)
  mlab.clf()
#  mlab.close(figm)

