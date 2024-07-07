//
// Created by guang on 2021/10/11.
//

#ifndef UTILS_H
#define UTILS_H

#include <chrono>
#include <fstream>
#include <sstream>
#include <map>
#include <numeric>
#include <string>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>

#include "common_lib.h"

namespace otd {

class KittiLoader{
public:
    KittiLoader(){}
    KittiLoader(const std::string &abs_path) {
        pc_path_ = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";
        posefile = abs_path + "/poses.txt";
        std::cout << "The file path is " << pc_path_ << std::endl;
        
        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % num_labels).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "\033[1;31mError: No files in " << pc_path_ << "\033[0m" << std::endl;
        }
        if (num_frames_ != num_labels) {
            std::cerr << "\033[1;31mError: The # of point clouds and # of labels are not same\033[0m" << std::endl;
        }
        std::cout << "Total " << num_frames_ << " files are loaded" << std::endl;
    }

    ~KittiLoader() {}

    size_t size() const { return num_frames_; }

    std::vector<Eigen::Matrix4d> get_poses()
    {
        std::vector<Eigen::Matrix4d> poses;
        std::ifstream ifs;
        ifs.open(posefile.c_str());
        std::string str;
        while(getline(ifs, str))
        {
            float * data = new float[12]{0.0}; 
            std::stringstream ss(str);
            std::string out;
            int i = 0;
            while (ss >> out)
            {
                float a = std::stof(out);    //string 转 float
                data[i] = a;    //将数据存放在临时数组中
                i++;
            }
            std::vector<double> pose(7, 0);
            Eigen::Matrix4d trans;
            trans <<  data[0], data[1], data[2], data[3],
                        data[4], data[5], data[6], data[7],
                        data[8], data[9], data[10], data[11],
                        0, 0, 0, 1;
            poses.push_back(trans);
        }

        ROS_ASSERT_MSG(poses.size() ==  num_frames_, "The pose num is error!!!!");
        return poses;
    }

    template<typename T>
    void get_cloud(size_t idx, pcl::PointCloud<T> &cloud) const {
        std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % idx).str();
        FILE *file = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
        }
        std::vector<float> buffer(1000000);
        size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        cloud.points.resize(num_points);
        if (std::is_same<T, pcl::PointXYZ>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
            }
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
            }
        } else if (std::is_same<T, PointXYZILI>::value) {
            std::string label_name = (boost::format("%s/%06d.label") % label_path_ % idx).str();
            std::ifstream label_input(label_name, std::ios::binary);
            if (!label_input.is_open()) {
                std::cerr << "error: failed to load " << label_name << std::endl;
            }
            label_input.seekg(0, std::ios::beg);
            std::vector<uint32_t> labels(num_points);
            label_input.read((char*)&labels[0], num_points * sizeof(uint32_t));

            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
                pt.label = labels[i] & 0xFFFF;
                pt.ring = idx;
            }
        }
    }

private:
    int num_frames_;
    std::string label_path_;
    std::string pc_path_;
    std::string posefile;
};

class PcdLoader{
public:
    PcdLoader(){}
    PcdLoader(const std::string &abs_path) {
        pc_path_ = abs_path + "/pcd";
        posefile = abs_path + "/poses.txt";

        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.pcd") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "\033[1;31mError: No files in " << pc_path_ << "\033[0m" << std::endl;
        }
        std::cout << "Total " << num_frames_ << " files are loaded" << std::endl;
    }

    ~PcdLoader() {}

    size_t size() const { return num_frames_; }

    void get_poses(std::vector<Eigen::Matrix4d> &poses, std::vector<double> &times)
    {
        std::ifstream ifs;
        ifs.open(posefile.c_str());
        std::string str;
        while(getline(ifs, str))
        {
            float * data = new float[8]{0.0}; 
            std::stringstream ss(str);
            std::string out;
            int i = 0;
            while (ss >> out)
            {
                float a = std::stof(out);    //string 转 float
                data[i] = a;    //将数据存放在临时数组中
                i++;
            }
            std::vector<double> pose(7, 0);
            Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
            Eigen::Matrix3d rot = q.toRotationMatrix();
            Eigen::Vector3d pos(data[1], data[2], data[3]);

            Eigen::Matrix4d tran = Eigen::Matrix4d::Identity();
            tran.topLeftCorner(3, 3) = rot;
            tran.topRightCorner(3, 1) = pos;
            poses.push_back(tran);
            times.push_back(data[0]);
        }

        ROS_ASSERT_MSG(poses.size() ==  num_frames_, "The pose num is error!!!!");
    }


    template<typename T>
    void get_cloud(size_t idx, pcl::PointCloud<T> &cloud) const 
    {
        std::string filename = (boost::format("%s/%06d.pcd") % pc_path_ % idx).str();
        if(pcl::io::loadPCDFile<T>(filename, cloud)==-1)
        {
            PCL_ERROR("Couldn't read file global_map.pcd\n");
            return;
        }
    }

private:
    int num_frames_;
    std::string label_path_;
    std::string pc_path_;
    std::string posefile;
};

}  // namespace otd

#endif  // UTILS_H
