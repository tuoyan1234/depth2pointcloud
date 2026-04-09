#include "depth2cloud.h"

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
        depth2pointCloud.depthImg2pointCloud(depthImgFiles, depth2pointCloud.cameraParams, depth2pointCloud.ply_save_path);
        std::cout << "深度图转点云完成" << std::endl;
    }else{
        std::cout << "未开启深度图转点云功能" << std::endl;
    }

    // 生成彩色点云
    if(depth2pointCloud.isDepthImg2ColorPC){
        depth2pointCloud.depthImg2ColorPC(depthImgFiles, rgbImgFiles, depth2pointCloud.cameraParams, depth2pointCloud.ply_save_path);
        std::cout << "深度图转彩色点云完成" << std::endl;
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
