import os.path 
import re
import subprocess as subp

name = "happySideRight_[0-9]+.ply"
name = "happyStandRight_[0-9]+.ply"
angle = 50.
translation = 10.
name = "[0-9]+_[0-9]_inv_depth_map_gray.ply"

name = "bun[0-9]+.ply"
angle = 90.
translation = 0.3

for root, dirs, files in os.walk("."):
  for file in files:
    if re.search(name, file):
      print file
      args = ['../../build/bin/rndTransformPc', 
          '-i '+file,
          '-o '+os.path.splitext(file)[0],
          '-t {}'.format(translation),
          '-a {}'.format(angle)
          ]
      err = subp.call(" ".join(args), shell=True)

