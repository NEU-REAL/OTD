#include <unistd.h>
#include <csignal>

#include "ground_segmentation.h"
#include "otd.h"
#include "common_lib.h"
#include "utils.hpp"

bool FLAG_EXIT = false;
void SigHandle(int sig) {
    FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "otd");
    ros::NodeHandle nh;

    int start_frame, end_frame;
    std::string dataset_path;
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    std::vector<double> retification_R_{9, 0.0};
    Eigen::Matrix4d T_i_l = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_reti = Eigen::Matrix4d::Identity();

    double sensor_height, tau_seeds, tau_dis;
    double tau_ratio, voxel_size;
    
    nh.param<std::string>("dataset/dataset_path", dataset_path, "");
    nh.param<int>("dataset/start_frame", start_frame, 0);
    nh.param<int>("dataset/end_frame", end_frame, -1);
    nh.param<std::vector<double>>("dataset/extrinsic_T", extrinT_, std::vector<double>());
    nh.param<std::vector<double>>("dataset/extrinsic_R", extrinR_, std::vector<double>());
    nh.param<std::vector<double>>("dataset/retification_R", retification_R_, std::vector<double>());
    
    nh.param<double>("groundseparate/sensor_height", sensor_height, 1.);
    nh.param<double>("groundseparate/tau_seeds", tau_seeds, 0.9);
    nh.param<double>("groundseparate/tau_dis", tau_dis, 0.1);

    nh.param<double>("otd/tau_ratio", tau_ratio, 0.7);
    nh.param<double>("otd/voxel_size", voxel_size, 0.5);

    std::cout << "The pointcloud path is " << dataset_path << std::endl;
    T_i_l.topLeftCorner(3, 3) = otd::common::MatFromArray<double>(extrinR_);
    T_i_l.topRightCorner(3, 1) = otd::common::VecFromArray<double>(extrinT_);
    T_reti.topLeftCorner(3, 3) = otd::common::MatFromArray<double>(retification_R_);
    std::cout << "The lidar_T_wrt_IMU is " << T_i_l << std::endl;
    std::cout << "The retification T is " << T_reti << std::endl;

    printf("The sensor height, tau_seeds and tau_dis is %lf, %lf, %lf\n", sensor_height, tau_seeds, tau_dis);
    printf("The otd tau_ratio and voxel size is %lf, %lf\n", tau_ratio, voxel_size);

    otd::KittiLoader loader(dataset_path);
    std::vector<Eigen::Matrix4d> poses = loader.get_poses();

    int max_frame = poses.size();
    std::cout << "The start end and max frame is " << start_frame << " " << end_frame << " " << max_frame << std::endl;
    ROS_ASSERT_MSG(max_frame > start_frame, "The start frame num is greater than the max frame num!!!!!!!!!!!");
    ROS_ASSERT_MSG((end_frame > start_frame), 
        "The start frame num is greater than the end frame num!!!!!!!!!!");

    if(end_frame > max_frame)
        end_frame = max_frame;

    auto p_grdseg_ = std::make_shared<otd::Grd_Seg<PointType>>(sensor_height, tau_seeds, tau_dis, true);
    auto p_otd_ = std::make_shared<otd::Otd3D<PointType>>(tau_ratio, voxel_size);

    std::cout << "The start and end frame num is " << start_frame << ", " << end_frame << std::endl;

    signal(SIGINT, SigHandle);    
    for (size_t i = start_frame; i < end_frame; i++)
    {
        if (FLAG_EXIT) {
            break;
        }
        std::cout << "The frame num is " << i << std::endl;
        PointCloudType::Ptr raw_ptr(new PointCloudType);
        PointCloudType::Ptr ground_ptr(new PointCloudType);
        PointCloudType::Ptr nonground_ptr(new PointCloudType);

        loader.get_cloud(i, *raw_ptr);
        Eigen::Matrix4d pose = T_reti * poses[i] * T_i_l;

        p_grdseg_->Run(raw_ptr, ground_ptr, nonground_ptr, pose);

        std::cout << "The ground and nonground points num is " << ground_ptr->size() << ", " << nonground_ptr->size() << std::endl;
        if (ground_ptr->empty() || nonground_ptr->empty())
            continue;

        p_otd_->Run(ground_ptr, nonground_ptr, i);
    }

    p_otd_->ComputeRateByKDTree({252, 253, 254, 255, 256, 257, 258, 259});
    
    p_grdseg_->GetTimeConsume();
    p_otd_->GetTimeConsume();

    return 0;
}
