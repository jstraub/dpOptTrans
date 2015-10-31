import subprocess as subp

datas = ['../data/middle.ply', '../data/left.ply', '../data/right.ply']

for data in datas:
  for overlap in [50, 60, 70, 80, 90]:
    args=["python", "./randomRenderAndCompare.py", 
        "-i " + data,
        "-m {}".format(overlap)]
    err = subp.call(" ".join(args), shell=True)
