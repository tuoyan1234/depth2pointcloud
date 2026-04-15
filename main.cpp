#include "depth2cloud.h"

// 提取目录下各个文件路径
void extractFilePath(const std::string& t_FileDir, std::vector<std::string>& t_FilePath){
    for (const auto& entry : std::filesystem::directory_iterator(t_FileDir))
    {
        if (!entry.is_regular_file())
            continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".ply" || ext == ".pcd")
        {
            std::string full_path = std::filesystem::absolute(entry.path()).string();
            t_FilePath.push_back(full_path);
        }
    }

    return ;
}



#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <thread>

// 显示点云
bool visPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    // 显示数据
    pcl::visualization::PCLVisualizer viewer;
    // 3. 创建两个视口
    int v1;
    // 设置背景颜色
    viewer.setBackgroundColor(0, 0, 0, v1);
    // 4. 添加点云（不同颜色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud_in, 0, 255, 0);   // 绿色
    viewer.addPointCloud<pcl::PointXYZ>(cloud_in, color_in, "input_cloud", v1);

    // 点大小
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud");
    // 5. 添加文字
    viewer.addText("Input Cloud", 10, 10, "v1 text", v1);
    // while(!viewer.wasStopped()){
    //     viewer.spinOnce(100);
    // }
    viewer.spinOnce(20);

    return true;
}
int main(){
    Depth2PointCloud depth2pointCloud;

    // 配置参数
    std::string ConfigFilePath = "../config/config.yaml";
    depth2pointCloud.readCameraParam(ConfigFilePath, depth2pointCloud);

    // 读取深度图
    std::vector<std::string> depthImgFiles = depth2pointCloud.readImage(depth2pointCloud.depthImgPath);
    // 读取rgb图片
    std::vector<std::string> rgbImgFiles = depth2pointCloud.readImage(depth2pointCloud.rgbImgPath);
    // 生成点云并保存
    if(depth2pointCloud.isDepthImg2PC){ 
        bool depth2pc_finished = depth2pointCloud.depthImg2pointCloud(depthImgFiles, depth2pointCloud.cameraParams, depth2pointCloud.ply_save_path);
        std::cout << "深度图转点云完成" << std::endl;
        if(false){
            // 读取文件路径
            std::vector<std::string> filesPath;
            extractFilePath(depth2pointCloud.ply_save_path, filesPath);
            std::sort(filesPath.begin(), filesPath.end());
            for(auto ply : filesPath){
                pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPLYFile(ply, *pointcloud_in);
                visPointcloud(pointcloud_in);
            }
        }
    }else{
        std::cout << "未开启深度图转点云功能" << std::endl;
    }

    // 生成彩色点云
    if(depth2pointCloud.isDepthImg2ColorPC){
        bool depthImg2ColorPC_finished = depth2pointCloud.depthImg2ColorPC(depthImgFiles, rgbImgFiles, depth2pointCloud.cameraParams, depth2pointCloud.colorPoints_save_path);
        std::cout << "深度图转彩色点云完成" << std::endl;
        std::vector<std::string> filesColorPCPath;
        extractFilePath(depth2pointCloud.colorPoints_save_path, filesColorPCPath);
        if(false){
            // 读取文件路径
            std::sort(filesColorPCPath.begin(), filesColorPCPath.end());
            // 读取文件路径
            std::sort(filesColorPCPath.begin(), filesColorPCPath.end());
            for(auto ply : filesColorPCPath){
                pcl::PointCloud<pcl::PointXYZ>::Ptr colorPC_in(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPLYFile(ply, *colorPC_in);
                visPointcloud(colorPC_in);
            }
            // system("pause");
        }else{ // TODO:fixme
            // 显示点云
            ///*
            pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
            while(!viewer.wasStopped()){
                for(auto file: filesColorPCPath){
                    PointCloud::Ptr cloud(new PointCloud);
                    if(pcl::io::loadPLYFile<DATA_TYPE>(file, *cloud) == -1){
                        PCL_ERROR("Couldn't read file %s \n", file.c_str());
                        return -1;
                    }
                    if(viewer.contains("VISCLOUD")){
                        viewer.removePointCloud("VISCLOUD");
                    }
                    viewer.addPointCloud(cloud, "VISCLOUD");
                    viewer.spinOnce(10);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
    }else{
        std::cout << "未开启深度图转彩色点云功能" << std::endl;
    }

    // 生成点云并投影到图片
    if(depth2pointCloud.isPcProject2Img){
        depth2pointCloud.depthImgProjetced2RgbImg(depthImgFiles, rgbImgFiles, depth2pointCloud.cameraParams, depth2pointCloud.projectedRgb_save_path,
        depth2pointCloud.depthpc2img_RT);
        std::cout << "点云投影到图像完成" << std::endl;
    }else{
        std::cout << "未开启点云投影到图像功能" << std::endl;
    }

    return 0;
}
