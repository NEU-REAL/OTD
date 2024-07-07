#include <unistd.h>
#include <csignal>
#include <mutex>

#include <ros/ros.h>
#include <ros/assert.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "ground_segmentation.h"
#include "otd.h"
#include "common_lib.h"
#include "utils.hpp"

std::queue<sensor_msgs::PointCloud2::ConstPtr> lidarQueue;
std::queue<nav_msgs::Odometry::ConstPtr> odomQueue;

std::mutex lidar_buf;
std::mutex odom_buf;

ros::Publisher ground_pub, nonground_pub, groundmap_pub, nongroundmap_pub, dynamicmap_pub, odom_pub, path_pub;
nav_msgs::Path path;

bool FLAG_EXIT = false;
void SigHandle(int sig) {
    FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

void PointCloud_callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg) 
{
    lidar_buf.lock();
    lidarQueue.push(lidar_msg);
    lidar_buf.unlock();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    odom_buf.lock();
    odomQueue.push(pose_msg);
    odom_buf.unlock();
}

bool sync_packages(double &t, Eigen::Matrix4d &pose, PointCloudType::Ptr &lidar_point)
{
    if (lidarQueue.empty() || odomQueue.empty()) {
        return false;
    }
    lidar_point->clear();
    sensor_msgs::PointCloud2::ConstPtr lidar_msg = lidarQueue.front();
    nav_msgs::Odometry::ConstPtr odom_msg = odomQueue.front();
    double t_lidar = lidar_msg->header.stamp.toSec();
    double t_odom = odom_msg->header.stamp.toSec();
    if (abs(t_lidar - t_odom) < 0.01)
    {
        t = t_odom;
        Eigen::Quaterniond odom_q;
        odom_q.w() = odom_msg->pose.pose.orientation.w;
        odom_q.x() = odom_msg->pose.pose.orientation.x;
        odom_q.y() = odom_msg->pose.pose.orientation.y;
        odom_q.z() = odom_msg->pose.pose.orientation.z;

        Eigen::Vector3d odom_t(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        pose.topLeftCorner(3, 3) = odom_q.toRotationMatrix();
        pose.topRightCorner(3, 1) = odom_t;

        pcl::PointCloud<pcl::PointXYZI> pcloud_temp;
        pcl::fromROSMsg(*lidar_msg, pcloud_temp);

        for (size_t i = 0; i < pcloud_temp.size(); i++)
        {
            PointType pt;
            pt.x = pcloud_temp.points[i].x;
            pt.y = pcloud_temp.points[i].y;
            pt.z = pcloud_temp.points[i].z;
            pt.intensity = int(pcloud_temp.points[i].intensity);
            pt.label = int((pcloud_temp.points[i].intensity - pt.intensity )* 100);
            // std::cout << pt.label << " ";
            lidar_point->push_back(pt);
        }
        // std::cout << std::endl;
        lidarQueue.pop();
        odomQueue.pop();
        return true;
    }
    else if(t_lidar - t_odom < 0)
    {
        lidarQueue.pop();
        std::cout << "The lidar msg is first!!!" << std::endl;
        return false;
    }
    else
    {
        odomQueue.pop();
        std::cout << "The odom msg is first!!!" << std::endl;
        return false;
    }
    
}

void publish_pointcloud(ros::Publisher ros_pub, PointCloudType::Ptr cloud_topub, double timestamp, std::string frame_id) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_topub, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
    ros_pub.publish(cloud_msg);
}

template<typename T>
void set_posestamp(T & out, Eigen::Matrix4d pose)
{
    Eigen::Matrix3d rot = pose.topLeftCorner(3, 3);
    Eigen::Vector3d pos = pose.topRightCorner(3, 1);
    Eigen::Quaterniond q(rot);

    out.pose.position.x = pos(0);
    out.pose.position.y = pos(1);
    out.pose.position.z = pos(2);
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
}

void publish_odom_path(Eigen::Matrix4d pose, double timestamp, std::string map_frame, std::string body_frame) {
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = ros::Time().fromSec(timestamp);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose, pose);
    odom_pub.publish(odomAftMapped);
    
    geometry_msgs::PoseStamped msg_body_pose;
    set_posestamp(msg_body_pose, pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(timestamp);
    msg_body_pose.header.frame_id = map_frame;
    path.poses.push_back(msg_body_pose);

    path.header.stamp = ros::Time().fromSec(timestamp);
    path.header.frame_id =map_frame;
    path_pub.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "otd");
    ros::NodeHandle nh;

    std::string lidar_topic, odom_topic;
    double stop_distance = -1;

    double sensor_height, tau_seeds, tau_dis;
    double tau_ratio, voxel_size;

    std::string save_path;

    Eigen::Vector3d pos_last = Eigen::Vector3d::Zero();
    int frame_num = 0;

    pcl::VoxelGrid<PointType> voxel_grid;
    double down_size;

    nh.param<std::string>("dataset/lid_topic",lidar_topic,"/cloud_registered_body");
    nh.param<std::string>("dataset/odom_topic", odom_topic,"/Odometry");
    nh.param<double>("dataset/stop_distance", stop_distance, -1);
    nh.param<double>("dataset/down_size", down_size, -1);
    
    nh.param<double>("groundseparate/sensor_height", sensor_height, 1.);
    nh.param<double>("groundseparate/tau_seeds", tau_seeds, 0.9);
    nh.param<double>("groundseparate/tau_dis", tau_dis, 0.1);

    nh.param<double>("otd/tau_ratio", tau_ratio, 0.7);
    nh.param<double>("otd/voxel_size", voxel_size, 0.5);

    nh.param<std::string>("save_path",save_path,"");

    std::cout << "The lidar topic is " << lidar_topic << std::endl;
    std::cout << "The odom topic is " << odom_topic << std::endl;
    if(down_size > 0) {
        voxel_grid.setLeafSize(down_size, down_size, down_size);
        std::cout << "The downsample size is " << down_size << std::endl;
    }

    printf("The sensor height, tau_seeds and tau_dis is %lf, %lf, %lf\n", sensor_height, tau_seeds, tau_dis);
    printf("The otd tau_ratio and voxel size is %lf, %lf\n", tau_ratio, voxel_size);

    std::cout << "The save path is " << save_path << std::endl;

    ros::Subscriber sub_LIDAR = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100000, PointCloud_callback);
    ros::Subscriber sub_ODOM = nh.subscribe<nav_msgs::Odometry>(odom_topic, 100000, odom_callback);

    ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_pc", 100);
    nonground_pub = nh.advertise<sensor_msgs::PointCloud2>("/nonground_pc", 100);
    groundmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_map", 100);
    nongroundmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/nonground_map", 100);
    dynamicmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_map", 100);
    path_pub = nh.advertise<nav_msgs::Path>("/path_otd", 100000);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry_otd", 100000);

    auto p_grdseg_ = std::make_shared<otd::Grd_Seg<PointType>>(sensor_height, tau_seeds, tau_dis);
    auto p_otd_ = std::make_shared<otd::Otd3D<PointType>>(tau_ratio, voxel_size);

    signal(SIGINT, SigHandle);
    ros::Rate rate(1000);

    while (ros::ok()) {
        if (FLAG_EXIT) {
            break;
        }
        ros::spinOnce();

        PointCloudType::Ptr raw_ptr(new PointCloudType);
        PointCloudType::Ptr ground_ptr(new PointCloudType);
        PointCloudType::Ptr nonground_ptr(new PointCloudType);
        Eigen::Matrix4d pose;
        double t;

        if(sync_packages(t, pose, raw_ptr))
        {
            std::cout << "The frame num is " << frame_num << std::endl;
            Eigen::Vector3d pos = pose.topRightCorner(3, 1);
            double pos_diff = (pos - pos_last).norm();
            if (pos_diff < stop_distance && stop_distance > 0)
            {
                std::cout << "The pos vec is too small, skip this frame!!!" << std::endl;
                continue;
            }
            pos_last = pos;

            p_grdseg_->Run(raw_ptr, ground_ptr, nonground_ptr, pose);

            std::cout << "The ground and nonground points num before downsample is " 
                << ground_ptr->size() << ", " << nonground_ptr->size() << std::endl;
            if (ground_ptr->empty() || nonground_ptr->empty())
                continue;
            publish_pointcloud(ground_pub, ground_ptr, t, "camera_init");
            publish_pointcloud(nonground_pub, nonground_ptr, t, "camera_init");

            voxel_grid.setInputCloud(ground_ptr);
            voxel_grid.filter(*ground_ptr);
            voxel_grid.setInputCloud(nonground_ptr);
            voxel_grid.filter(*nonground_ptr);

            std::cout << "The ground and nonground points num after downsample is " 
                << ground_ptr->size() << ", " << nonground_ptr->size() << std::endl;

            p_otd_->Run(ground_ptr, nonground_ptr, frame_num);

            publish_odom_path(pose, t, "camera_init", "body");

            if(frame_num % 500 == 1) {
                p_otd_->PruningVoxel(100000);
            }

            if(frame_num % 20 == 1) {
                PointCloudType::Ptr ground_topub(new PointCloudType);
                PointCloudType::Ptr nonground_topub(new PointCloudType);
                PointCloudType::Ptr dynamic_topub(new PointCloudType);
                p_otd_->GetMap(ground_topub, nonground_topub, dynamic_topub);

                publish_pointcloud(groundmap_pub, ground_topub, t, "camera_init");
                publish_pointcloud(nongroundmap_pub, nonground_topub, t, "camera_init");
                publish_pointcloud(dynamicmap_pub, dynamic_topub, t, "camera_init");
            }
        }

        frame_num++;

        rate.sleep();
    }

    p_otd_->SaveMap(save_path);

    return 0;
}
