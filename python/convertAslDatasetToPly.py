import numpy as np
import os, re
from js.data.plyParse import PlyParse
import subprocess as subp

folder = "../data/stairs/"
folder = "../data/wood_summer/"
folder = "../data/apartment/"
folder = "../data/mountain_plain/"
folder = "../data/gazebo_winter/"
folder = "../data/gazebo_summer/"
name = "Hokuyo_[0-9]+.csv"
paths = []
for root, dirs, files in os.walk(folder):
  for f in files:
    if re.search(name, f):
      paths.append(os.path.join(root,f))

paths = sorted(paths, key=lambda f: 
        int(re.sub(".csv","",re.sub("Hokuyo_","",os.path.split(f)[1]))))

#paths = paths[36:38]

for path in paths:
  print path
  with open(path) as f:
    print f.readline()
    x = np.loadtxt(f,delimiter=",")
    pc = x[:,1:4]
  ply = re.sub("csv","ply",path)
  with open(ply,"w") as f:
    f.write("ply \n")
    f.write("format ascii 1.0 \n")
    f.write("comment generated for asl robot data \n")
    f.write("element vertex {} \n".format(pc.shape[0]))
    f.write("property float x \n")
    f.write("property float y \n")
    f.write("property float z \n")
    f.write("end_header\n")
    np.savetxt(f, pc, fmt='%.9f %.9f %.9f')
#    for i in range(pc.shape[0]):
#      f.write("{} {} {}\n".format(pc[i,0],pc[i,1],pc[i,2]))

  args = ['../cudaPcl/pod-build/bin/pclNormals', 
      '-i {}'.format(os.path.abspath(ply)), 
      '-o ' + os.path.abspath(re.sub("Hokuyo","HokuyoPcNormals_0_2",ply)),
      '-s 0.2'
      ]
  print " ".join(args)
  err = subp.call(" ".join(args), shell=True)
  
