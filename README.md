### Optimal Point Cloud Alignment 

This package implements optimal point cloud alignment of two given
point clouds. It relies on the following libraries which can be found
on my github as well:
[jsCore](https://github.com/jstraub/jsCore)
[cudaPcl](https://github.com/jstraub/cudaPcl)
[dpMMlowVar](https://github.com/jstraub/dpMMlowVar)
[bbTrans](https://github.com/jstraub/bbTrans).
The approach and the algorithm is described in more detail on our
[project page](http://www.jstraub.de/) and the [CVPR 2016 paper](http://www.jstraub.de/). The algorithms are described in more detail in the [supplement](http://www.jstraub.de/download/).
See below for install instructions.

If you use it please cite:
```
Julian Straub, Trevor Campbell, Jonathan P. How, John W. Fisher III. 
"Optimal Point Cloud Alignment", In CVPR,
2016.
```

### Dependencies
This code is dependent on Eigen3, Boost, CUDA, OpenCV and PCL.
It has been tested on Ubuntu 14.04 with 
- Eigen3 (3.0.5) 
- Boost (1.54)
- CUDA (6.5)
- OpenCV (2.4)
- PCL (1.7)

### Install

This package uses [the pods build
system](http://sourceforge.net/p/pods/home/Home/). Used widely at CSAIL
MIT the build system makes it easy to break up software projects into
small packages that can be checked out and compiled automatically (see
below).

- *Linux:* 

    Install Eigen3, Boost, OpenCV, and PCL

    ```
    sudo apt-get install libeigen3-dev libboost-dev libopencv-dev libpcl-1.7-all-dev
    ```

    Install the appropriate CUDA version matching with your nvidia
    drivers. On our machines we use `nvidia-340-dev` with
    `libcuda1-340 cuda-6-5 cuda-toolkit-6-5`

    Clone this repository and compile the code:

    ```
    git clone git@github.mit.edu:jstraub/dpOptTrans.git; cd dpOptTrans
    make checkout; make configure; make -j6; make install;
    ```
    
    Note that this will checkout several other necessary repositories.
    To update all repositories run
    
    ```
    make update; make configure; make -j6; make install;
    ```

### Getting Started

To run the BB algorithm on two point clouds run a command like this:
```
cd build/bin/; ./dpOptTransPly -l 60 -t 0.001 -a ../../data/bunny_rnd/bun_zipper.ply -b ../../data/bunny_rnd/bun_zipper_angle_90_translation_0.3.ply -o test
```
This will generate a outputfile test.csv which contains the optimal rotation as
a quaternion, the optimal translation, the lower bounds for rotation as well as
translation and the number of clusters used for the alignment in surface normal
as well as Euclidean space. So for the command above you should get an output
file containing the following:
```
q_w q_x q_y q_z t_x t_y t_z lb_S3 lb_R3 KvmfA KvmfB KgmmA KgmmB K
0.707106 0.381209 -0.0583649 0.592684 0.254969 -0.144869 -0.0632762 0.114808 184.636 8 8 35 35 1
```

To get additional visualization for the result try running the python script:
```
cd python; python alignBuddha.py
```

The methods in python/helpers.py show how to form the arguments of all provided executables.

### Usage 

```
 ./dpOptTransPly -h
Allowed options:
  -h [ --help ]          produce help message
  -l [ --lambdaDeg ] arg lambda in degree for DPvMF-means
  -t [ --lambdaT ] arg   lambda in meters for DPMeans
  -a [ --in_a ] arg      path to first input file
  -b [ --in_b ] arg      path to second input file
  -o [ --out ] arg       path to output file
  -s [ --scale ] arg     scale for point-cloud
  -e [ --egi ]           make the vMF MM pis uniform - like a EGI
  -d [ --display ]       display results
  -v [ --verbose ]       be verbose
``` 
