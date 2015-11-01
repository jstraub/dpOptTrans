#!/bin/bash
cd ../../3rdparty/egi_constellation/
matlab -nojvm -nodesktop -r $'compute_Rt_egi(\''$1$'\',\''$2$'\',\''$3$'\')'

