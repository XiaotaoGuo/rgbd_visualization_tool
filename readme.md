#RGB_visualization_tool
---
a tool for visualizing RGB-D image in 3-D
###dependency
opencv
Eigen
Pangolin

###usage
mkdir build
cd build
cmake ..
make
./3dreconstruct ([path for rgb image]) [path for depth image]

###example
depth only
./3dreconstruct ../example/71.png

###others
intrinsic and the way to read image could be changed in rgb_visualize.cpp

