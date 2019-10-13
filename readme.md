依赖库
opencv
Eigen
Pangolin

使用
mkdir build
cd build
cmake ..
make
./3dreconstruct ([rgb图路径]) [深度图路径]

example
./3dreconstruct ../exmaple/89.jpg ../example/89.png
只使用深度图
./3dreconstruct ../example/90.png

内参和图片读取方式可以在rgbd_visualize.cpp修改

