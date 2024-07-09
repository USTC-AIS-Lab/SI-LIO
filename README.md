SI_LIO is based on S-FAST_LIO and FAST_LIO 

## 1. Dependices
Sophus
Eigen
PCL
livox_ros_driver

## 2. Build SI-LIO
Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/USTC-AIS-Lab/SI-LIO.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. Run:
```
roslaunch inva_fast_lio mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 4. Acknowledgements
Thanks for the authors of [S-FAST-LIO](https://github.com/zlwang7/S-FAST_LIO.git) and [FAST-LIO](https://github.com/hku-mars/FAST_LIO).

