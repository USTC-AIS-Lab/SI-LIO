# SI_LIO
**SI_LIO** is based on the Invariant-EKF theory and our code is implemented on S-FAST_LIO.
<p align="center" style="display: flex; justify-content: center;">
    <img src="./img/gate_03.png" alt="drawing" width="350"/>
    <img src="./img/gate_03_compare.png" alt="drawing" width="350"/>
</p>

Through theoretical analysis, it can be deduced that our IEKF-based estimation method achieves higher estimation accuracy compared to the Iterated EKF used in FAST-LIO2, particularly in scenarios with large IMU prediction errors. This experimental outcome corroborates our analytical conclusions.

## 1. Dependices
Sophus
Eigen
PCL
livox_ros_driver

## 2. Build
Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/USTC-AIS-Lab/SI-LIO.git
cd ../
catkin_make -j
source ~/catkin_ws/devel/setup.bash
```

## 3. Run
We recommend using the [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) dataset.
```
roslaunch inva_fast_lio mapping_velodyne_m2dgr.launch
rosbag play street_04.bag
```

## 4. Acknowledgements
Thanks for the authors of [S-FAST-LIO](https://github.com/zlwang7/S-FAST_LIO.git) and [FAST-LIO](https://github.com/hku-mars/FAST_LIO).

