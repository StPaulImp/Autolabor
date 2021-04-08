# Autolabor
# shtd_autolabor
# 总体目标： 

1. 使用速腾16线激光雷达替代autolabor双单线激光。 
2. 使用cartogragher建图(3D),定位。

# 编译：

1. $cd catkin_ws
2. $catkin config --no-install --no-cmake-args
3. $catkin build -j10

# 更正编译时python环境错误

1. sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 150
2. sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 100

# libfreenect2
1. catkin config --cmake-args -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
2. catkin build