#ifndef DEPTH_2_POINTCLOUD_H
#define DEPTH_2_POINTCLOUD_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>

class Depth2PointCloud
{
private:

public:
    // 相机内参结构体（含深度/彩色相机内参 + 畸变系数）
    struct cameraParam{
        // depth camera intrinsic parameter
        double dfx = 0.0;
        double dfy = 0.0;
        double dcx = 0.0;
        double dcy = 0.0;

        // rgb camera intrinsic parameter
        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;

        // 相机畸变系数
        double k1 = 0.0;
        double k2 = 0.0;
        double p1 = 0.0;
        double p2 = 0.0;
        double k3 = 0.0;
    };

    // 路径配置参数
    std::string depthImgPath, rgbImgPath;
    std::string ply_save_path;
    std::string projectedRgb_save_path;
    std::string ConfigFilePath;

    // 核心参数对象
    cameraParam cameraParams;
    Eigen::Matrix4d depthpc2img_RT = Eigen::Matrix4d::Identity(); // 深度相机到RGB相机的外参变换矩阵

    // 控制转换内容开关
    bool isDepthImg2PC = false;        // 深度图转普通点云
    bool isDepthImg2ColorPC = false;   // 深度图转彩色点云
    bool isPcProject2Img = false;      // 点云投影到图像

    // 构造/析构函数
    Depth2PointCloud(/* args */);
    ~Depth2PointCloud();

    // 成员函数声明
    bool readCameraParam(const std::string& t_cameraConfigFilePath, Depth2PointCloud& t_depth2pointCloud);
    std::vector<std::string> readImage(const std::string& t_depthFileDirs);

    // depth2pc 核心功能函数
    void depthImg2pointCloud(std::vector<std::string>& depthFileDirs, const cameraParam& t_cameraParams, const std::string& t_ply_save_path);
    void depthImg2ColorPC(std::vector<std::string>& depthFileDirs, std::vector<std::string>& rgbFileDirs, const cameraParam& t_cameraParams, const std::string& t_ply_save_path);
    // pcProject2Img 点云投影功能函数
    void depthImgProjetced2RgbImg(std::vector<std::string>& depthFileDirs, std::vector<std::string>& rgbFileDirs, const cameraParam& t_cameraParams, const std::string& t_projectedRgb_save_path, const Eigen::Matrix4d& t_depthpc2img_RT);
};
#endif