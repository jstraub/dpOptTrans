import numpy as np
from scipy.linalg import solve
from js.geometry.quaternion import Quaternion


for path in [ "../data/gazebo_winter/", "../data/mountain_plain/", "../data/gazebo_summer/" ]:
#for path in [ "../data/stairs/", "../data/apartment/", "../data/wood_summer/" ]:
  with open(path+"pose_scanner_leica.csv") as f:
    f.readline()
    x = np.loadtxt(f,delimiter=",")
    for i in range(x.shape[0]):
      T_wc = np.reshape(x[i,2:],(4,4))
      R_wc = T_wc[:3,:3]
      q_wc = Quaternion() 
      q_wc.fromRot3(R_wc)
      t_wc = T_wc[:3,3]
      print t_wc, q_wc
      with open(path+"pose_{}.csv".format(i),"w") as fout:
        fout.write("q_w q_x q_y q_z t_x t_y t_z\n")
        fout.write("{} {} {} {} {} {} {}".format(q_wc.q[0],\
            q_wc.q[1],q_wc.q[2],q_wc.q[3],t_wc[0],t_wc[1],t_wc[2]))
