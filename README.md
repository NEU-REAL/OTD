## OTD
**OTD** (Observation Time Difference) an online dynamic traces removal package. It takes voxels as the smallest unit for dynamic traces removal, and is based on the assumption that static voxels always appear and disappear simultaneously with the ground below them. Therefore, the voxel that appears later than the ground as suddenly appear dynamic voxel, and voxel that disappears earlier than the ground as suddenly disappear dynamic voxel. We call this method of judging dynamic voxels as observation time difference.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for OTD to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2 **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.3 **BOOST**
OTD needs boost to load KITTI dataset.

## 2. Build from source
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone git@github.com:RongguangWu/OTD.git
    catkin_make
    source devel/setup.bash
```

## 3. Directly run
### 3.1 For KITTI
1. Download the KITTI dataset from the SemanticKITTI official website (http://www.semantic-kitti.org/index.html).
2. Modify the dataset_path, start_frame, and end_frame parameters in the yaml file under the config folder.
3. Run
```
    cd ~/$A_ROS_DIR$
    source devel/setup.bash
    roslaunch otd otd_kitti.launch
```
### 3.2 For your own bag file
Your bag file needs to contain the LiDAR point cloud topic (sensor_msgs/PointCloud2) and the odometry topic (nav_msgs/Odometry). The point cloud is in the local coordinate frame, while the odometry is the transformation from the local frame to the global frame.

**Special note: OTD requires ground segmentation, and the ground segmentation method we use requires that the height of the LiDAR from the ground is fixed. Therefore, for handheld or airborne devices, our method will not work.**

1. Modify the lid_topic, odom_topic, and sensor_height(**The height of the origin of the local coordinate frame from the ground**) parameters in the online.yaml file under the config folder.
2. Run
```
    cd ~/$A_ROS_DIR$
    source devel/setup.bash
    roslaunch otd otd_online.launch
```

## 6.Acknowledgments
Thanks for the iVox from Faster-LIO(C. Bai, T. Xiao, Y. Chen, H. Wang, F. Zhang, and X. Gao. Faster-lio: Lightweight tightly coupled lidar-inertial odometry using parallel sparse incremental voxels).
