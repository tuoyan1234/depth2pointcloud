#include "depth2cloud.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

// 构造函数
Depth2PointCloud::Depth2PointCloud(/* args */)
{
}

// 析构函数
Depth2PointCloud::~Depth2PointCloud()
{
}

// 读取相机参数配置文件（YAML格式）
bool Depth2PointCloud::readCameraParam(const std::string& t_cameraConfigFilePath, Depth2PointCloud& t_depth2pointCloud){
    // 加载YAML配置文件
    YAML::Node params_node = YAML::LoadFile(t_cameraConfigFilePath);

    // 1. 读取RGB相机内参 & 畸变系数
    // fx
    if(params_node["color_camera_intrinsic_params"]["fx"].IsDefined()){
        t_depth2pointCloud.cameraParams.fx = params_node["color_camera_intrinsic_params"]["fx"].as<double>();
    }else{
        std::cout << "fx is not exist" << std::endl;
        return false;
    }
    // fy
    if(params_node["color_camera_intrinsic_params"]["fy"].IsDefined()){
        t_depth2pointCloud.cameraParams.fy = params_node["color_camera_intrinsic_params"]["fy"].as<double>();
    }else{
        std::cout << "fy is not exist" << std::endl;
        return false;
    }
    // cx
    if(params_node["color_camera_intrinsic_params"]["cx"].IsDefined()){
        t_depth2pointCloud.cameraParams.cx = params_node["color_camera_intrinsic_params"]["cx"].as<double>();
    }else{
        std::cout << "cx is not exist" << std::endl;
        return false;
    }
    // cy
    if(params_node["color_camera_intrinsic_params"]["cy"].IsDefined()){
        t_depth2pointCloud.cameraParams.cy = params_node["color_camera_intrinsic_params"]["cy"].as<double>();
    }else{
        std::cout << "cy is not exist" << std::endl;
        return false;
    }

    // RGB畸变系数 (k1, k2, p1, p2, k3)
    std::vector<double> rgbCoeffs(5, 0.0); // 修正变量名，移除非法空格
    if(params_node["color_camera_intrinsic_params"]["Coeffs"].IsDefined()){
        rgbCoeffs = params_node["color_camera_intrinsic_params"]["Coeffs"].as<std::vector<double>>();
        // 赋值给结构体成员
        t_depth2pointCloud.cameraParams.k1 = rgbCoeffs[0];
        t_depth2pointCloud.cameraParams.k2 = rgbCoeffs[1];
        t_depth2pointCloud.cameraParams.p1 = rgbCoeffs[2];
        t_depth2pointCloud.cameraParams.p2 = rgbCoeffs[3];
        t_depth2pointCloud.cameraParams.k3 = rgbCoeffs[4];
    }else{
        std::cout << "rgb Coeffs is not exist" << std::endl;
        return false;
    }

    // 2. 读取深度相机内参
    // dfx
    if(params_node["depth_camera_intrinsic_params"]["dfx"].IsDefined()){
        t_depth2pointCloud.cameraParams.dfx = params_node["depth_camera_intrinsic_params"]["dfx"].as<double>();
    }else{
        std::cout << "dfx is not exist" << std::endl;
        return false;
    }
    // dfy
    if(params_node["depth_camera_intrinsic_params"]["dfy"].IsDefined()){
        t_depth2pointCloud.cameraParams.dfy = params_node["depth_camera_intrinsic_params"]["dfy"].as<double>();
    }else{
        std::cout << "dfy is not exist" << std::endl;
        return false;
    }
    // dcx
    if(params_node["depth_camera_intrinsic_params"]["dcx"].IsDefined()){
        t_depth2pointCloud.cameraParams.dcx = params_node["depth_camera_intrinsic_params"]["dcx"].as<double>();
    }else{
        std::cout << "dcx is not exist" << std::endl;
        return false;
    }
    // dcy
    if(params_node["depth_camera_intrinsic_params"]["dcy"].IsDefined()){
        t_depth2pointCloud.cameraParams.dcy = params_node["depth_camera_intrinsic_params"]["dcy"].as<double>();
    }else{
        std::cout << "dcy is not exist" << std::endl;
        return false;
    }

    // 3. 读取路径配置
    // depthImgPath
    if(params_node["depthImgPath"].IsDefined()){
        t_depth2pointCloud.depthImgPath = params_node["depthImgPath"].as<std::string>();
    }else{
        std::cout << "depthImgPath is not exist" << std::endl;
        return false;
    }
    // rgbImgPath
    if(params_node["rgbImgPath"].IsDefined()){
        t_depth2pointCloud.rgbImgPath = params_node["rgbImgPath"].as<std::string>();
    }else{
        std::cout << "rgbImgPath is not exist" << std::endl;
        return false;
    }
    // ply save path
    if(params_node["ply_save_path"].IsDefined()){
        t_depth2pointCloud.ply_save_path = params_node["ply_save_path"].as<std::string>();
        // if directory not exist, create it
        if(!boost::filesystem::exists(t_depth2pointCloud.ply_save_path)){
            boost::filesystem::create_directories(t_depth2pointCloud.ply_save_path);
            std::cout << "ply_save_path is created" << std::endl;
        }else{
            std::cout << "ply_save_path already exist !" << std::endl;
        }
    }else{
        std::cout << "ply_save_path is not exist" << std::endl;
        return false;
    }
    
    // projectedRgb_save_path
    if(params_node["projectedRgb_save_path"].IsDefined()){
        t_depth2pointCloud.projectedRgb_save_path = params_node["projectedRgb_save_path"].as<std::string>();
        // if directory not exist, create it
        if(!boost::filesystem::exists(t_depth2pointCloud.projectedRgb_save_path)){
            boost::filesystem::create_directories(t_depth2pointCloud.projectedRgb_save_path);
            std::cout << "projectedRgb_save_path is created" << std::endl;
        }else{
            std::cout << "projectedRgb_save_path already exist !" << std::endl;
        }
    }else{
        std::cout << "projectedRgb_save_path is not exist" << std::endl;
        return false;
    }
    
    // isDepthImg2PC
    if(params_node["isDepthImg2PC"].IsDefined()){
        t_depth2pointCloud.isDepthImg2PC = params_node["isDepthImg2PC"].as<bool>();
    }else{
        std::cout << "isDepthImg2PC is not exist" << std::endl;
        return false;
    }
    
    // isDepthImg2ColorPC
    if(params_node["isDepthImg2ColorPC"].IsDefined()){
        t_depth2pointCloud.isDepthImg2ColorPC = params_node["isDepthImg2ColorPC"].as<bool>();
    }else{
        std::cout << "isDepthImg2ColorPC is not exist" << std::endl;
        return false;
    }
    
    // isPcProject2Img
    if(params_node["isPcProject2Img"].IsDefined()){
        t_depth2pointCloud.isPcProject2Img = params_node["isPcProject2Img"].as<bool>();
    }else{
        std::cout << "isPcProject2Img is not exist" << std::endl;
        return false;
    }
    // depthpc2img params
    std::vector<double> depthpc2img_R_params(9, 0.0);
    std::vector<double> depthpc2img_T_params(16, 0.0);

    if(params_node["depthPC2RGB_params"].IsDefined()){
        depthpc2img_R_params = params_node["depthPC2RGB_params"]["R_lidar2camera"].as<std::vector<double>>();
        depthpc2img_T_params = params_node["depthPC2RGB_params"]["T_lidar2camera"].as<std::vector<double>>();
    }else{
        std::cout << "depthPC2RGB_params is not exist" << std::endl;
        return false;
    }

    // 将读取的深度相机到彩色相机的旋转矩阵赋值给转换变量
    t_depth2pointCloud.depthpc2img_RT.block<3,3>(0,0) = Eigen::Map<Eigen::Matrix3d>(depthpc2img_R_params.data());
    t_depth2pointCloud.depthpc2img_RT(0,3) = depthpc2img_T_params[3];
    t_depth2pointCloud.depthpc2img_RT(1,3) = depthpc2img_T_params[7];
    t_depth2pointCloud.depthpc2img_RT(2,3) = depthpc2img_T_params[11];

    return true;
}

std::vector<std::string> Depth2PointCloud::readImage(const std::string& t_depthFileDirs){
    std::vector<std::string> depthImageFiles;

    for (const auto& entry : std::filesystem::directory_iterator(t_depthFileDirs))
    {
        if (!entry.is_regular_file())
            continue;

        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg")
        {
            std::string full_path = std::filesystem::absolute(entry.path()).string();
            depthImageFiles.push_back(full_path);
        }
    }

    return depthImageFiles;
}

void Depth2PointCloud::depthImg2pointCloud(std::vector<std::string>& depthFileDirs, const Depth2PointCloud::cameraParam& t_cameraParams, const std::string& t_ply_save_path) {
    std::sort(depthFileDirs.begin(), depthFileDirs.end());
    for(auto depthFile: depthFileDirs){
        std::string ply_stem = std::filesystem::path(depthFile).stem().string();
        cv::Mat depthImage = cv::imread(depthFile,cv::IMREAD_UNCHANGED);
        // 深度图转换点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        int imageWidth = depthImage.cols;
        int imageHeight = depthImage.rows;

        int bias_row = 0;
        int bias_col = 0;
        for (int index_row = 0; index_row < imageHeight; index_row += (2+bias_row)){
            if (index_row > imageHeight * 0.9) {
                bias_row = 3;
                bias_col = 3;
            } else if (index_row > imageHeight * 0.8) {
                bias_row = 2;
                bias_col = 2;
            } else if (index_row > imageHeight * 0.7) { // 深度图越下方离车越近，降采样倍率越高 indY:108开始
                bias_row = 1;
                bias_col = 1;
            }
            for(int index_col = 0; index_col < imageWidth; index_col += (3+bias_col)){
                pcl::PointXYZ point;

                point.y = depthImage.at<uint16_t>(index_row,index_col) / 1000.0;
                point.x = (index_col - t_cameraParams.dcx) * point.y / t_cameraParams.dfx;
                point.z = -(index_row - t_cameraParams.dcy) * point.y / t_cameraParams.dfy;
                pointCloud->push_back(point);
            }
        }
        std::string save_plyFilePath = t_ply_save_path + ply_stem + ".ply";
        pcl::io::savePLYFileBinary(save_plyFilePath, *pointCloud);
    }
}

// 彩色点云转换
void Depth2PointCloud::depthImg2ColorPC(std::vector<std::string>& depthFileDirs,std::vector<std::string>& rgbFileDirs,const cameraParam& t_cameraParams,const std::string& t_ply_save_path){
    std::sort(depthFileDirs.begin(), depthFileDirs.end());
    std::sort(rgbFileDirs.begin(), rgbFileDirs.end());
    for(int img_index = 0; img_index < depthFileDirs.size(); img_index++){
        std::string ply_stem = std::filesystem::path(depthFileDirs[img_index]).stem().string();
        cv::Mat depthImage = cv::imread(depthFileDirs[img_index],cv::IMREAD_UNCHANGED);
        cv::Mat rgbImage = cv::imread(rgbFileDirs[img_index],cv::IMREAD_UNCHANGED);
        // 深度图转换点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
        int imageWidth = depthImage.cols;
        int imageHeight = depthImage.rows;

        int bias_row = 0;
        int bias_col = 0;
        for (int index_row = 0; index_row < imageHeight; index_row += (2+bias_row)){
            if (index_row > imageHeight * 0.9) {
                bias_row = 3;
                bias_col = 3;
            } else if (index_row > imageHeight * 0.8) {
                bias_row = 2;
                bias_col = 2;
            } else if (index_row > imageHeight * 0.7) { // 深度图越下方离车越近，降采样倍率越高 indY:108开始
                bias_row = 1;
                bias_col = 1;
            }
            for(int index_col = 0; index_col < imageWidth; index_col += (3+bias_col)){
                pcl::PointXYZRGB point;

                point.y = depthImage.at<uint16_t>(index_row,index_col) / 1000.0;
                point.x = (index_col - t_cameraParams.dcx) * point.y / t_cameraParams.dfx;
                point.z = -(index_row - t_cameraParams.dcy) * point.y / t_cameraParams.dfy;
                point.b = rgbImage.at<cv::Vec3b>(index_row, index_col)[0];
                point.g = rgbImage.at<cv::Vec3b>(index_row, index_col)[1];
                point.r = rgbImage.at<cv::Vec3b>(index_row, index_col)[2];
                pointCloudRGB->push_back(point);
            }
        }
        std::string save_plyFilePath = t_ply_save_path + ply_stem + ".ply";
        pcl::io::savePLYFileBinary(save_plyFilePath, *pointCloudRGB);
    }
}

//
// pcProject2Img
// 彩色点云转换 depthImg2ColorPC
void Depth2PointCloud::depthImgProjetced2RgbImg(std::vector<std::string>& depthFileDirs,std::vector<std::string>& rgbFileDirs,const cameraParam& t_cameraParams,const std::string& t_projectedRgb_save_path,const Eigen::Matrix4d& t_depthpc2img_RT){
    std::sort(depthFileDirs.begin(),depthFileDirs.end());
    std::sort(rgbFileDirs.begin(),rgbFileDirs.end());
    for(int img_index = 0; img_index < depthFileDirs.size(); img_index++){
        std::string ply_stem = std::filesystem::path(depthFileDirs[img_index]).stem().string();
        cv::Mat depthImage = cv::imread(depthFileDirs[img_index],cv::IMREAD_UNCHANGED);
        cv::Mat rgbImage = cv::imread(rgbFileDirs[img_index],cv::IMREAD_UNCHANGED);

        // 深度图转换点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
        int imageWidth = depthImage.cols;
        int imageHeight = depthImage.rows;

        int bias_row = 0;
        int bias_col = 0;
        for (int index_row = 0; index_row < imageHeight; index_row += (2+bias_row)){
            if (index_row > imageHeight * 0.9) {
                bias_row = 3;
                bias_col = 3;
            } else if (index_row > imageHeight * 0.8) {
                bias_row = 2;
                bias_col = 2;
            } else if (index_row > imageHeight * 0.7) { // 深度图越下方离车越近，降采样倍率越高 indY:108开始
                bias_row = 1;
                bias_col = 1;
            }
            for(int index_col = 0; index_col < imageWidth; index_col += (3+bias_col)){
                pcl::PointXYZRGB point;

                point.y = depthImage.at<uint16_t>(index_row,index_col) / 1000.0;
                point.x = (index_col - t_cameraParams.dcx) * point.y / t_cameraParams.dfx;
                point.z = -(index_row - t_cameraParams.dcy) * point.y / t_cameraParams.dfy;

                pcl::PointXYZRGB temp_point;
                // 先对x =x, y = -z, z = y进行坐标变换
                temp_point.x = point.x;
                temp_point.y = -point.z;
                temp_point.z = point.y;
                temp_point.b = rgbImage.at<cv::Vec3b>(index_row,index_col)[0];
                temp_point.g = rgbImage.at<cv::Vec3b>(index_row,index_col)[1];
                temp_point.r = rgbImage.at<cv::Vec3b>(index_row,index_col)[2];
                pointCloudRGB->push_back(temp_point);
            }
        }
        // 点云转换到相机坐标系
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudRGB_cambase(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*pointCloudRGB, *pointCloudRGB_cambase, t_depthpc2img_RT);

        // 点云、图像对齐
        for(auto point : *pointCloudRGB_cambase){
            if(point.z < 0){
                continue;
            }else{
                double X = point.x / point.z;
                double Y = point.y / point.z;
                double R2 = X * X + Y * Y;
                double R4 = R2 * R2;
                double R6 = R2 * R4;

                // 径向畸变depthImage
                double XR = X * (1 + t_cameraParams.k1 * R2 + t_cameraParams.k2 * R4 + t_cameraParams.k3 * R6);
                double YR = Y * (1 + t_cameraParams.k1 * R2 + t_cameraParams.k2 * R4 + t_cameraParams.k3 * R6);

                // 切向畸变
                double XD = XR + 2 * t_cameraParams.p1 * X * Y + t_cameraParams.p2 * (R2 + 2 * X * X);
                double YD = YR + t_cameraParams.p1 * (R2 + 2 * Y * Y) + 2 * t_cameraParams.p2 * X * Y;

                // 像素坐标
                // int u = std::round(t_cameraParams.fx * XD + t_cameraParams.cx);
                // int v = std::round(t_cameraParams.fy * YD + t_cameraParams.cy);
                int u = (t_cameraParams.fx * XD + t_cameraParams.cx);
                int v = (t_cameraParams.fy * YD + t_cameraParams.cy);

                // 像素是否在图像内
                if(u >= 0 && u < rgbImage.cols && v >= 0 && v < rgbImage.rows){
                    cv::circle(rgbImage, cv::Point(u, v), 1, cv::Scalar(0, 0, 255), -1);
                }
            }
        }

        // 保存投影结果
        std::string projected_rgb_name = t_projectedRgb_save_path + ply_stem + ".jpg";
        cv::imwrite(projected_rgb_name, rgbImage);
    }
        
    return;
}
        






