import os.path 
import re
import subprocess as subp

angle = 90.
translation = 0.3
name = "happySideRight_[0-9]+.ply"
name = "bun[0-9]+.ply"

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

