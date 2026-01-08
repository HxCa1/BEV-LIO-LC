#include "BEVprojector.h"
#include <stdexcept>
#define DUPLICATE_POINTS 10

BEVProjector::BEVProjector(ros::NodeHandle nh) {
    try {
        loadParameters(nh);
    } catch (const std::runtime_error& e) {
        ROS_ERROR_STREAM(e.what());
        exit(1);
    }
}

void BEVProjector::loadParameters(ros::NodeHandle nh) {
    nh.param("bev/resolution", resolution_, 0.1);              // 分辨率，单位为米
    nh.param("bev/max_x_", max_x_, 50.0);                      // 最大 x 坐标
    nh.param("bev/min_x_", min_x_, -50.0);                     // 最小 x 坐标
    nh.param("bev/max_y_", max_y_, 50.0);                      // 最大 y 坐标
    nh.param("bev/min_y_", min_y_, -50.0);                     // 最小 y 坐标
    nh.param("bev/voxel_size", voxel_size_, 0.4);              // 体素大小
    nh.param("bev/downsample",downsample_,true);               // 图像垂直方向上栅格数
    nh.param("bev/normalize_to_255", normalize_to_255_, true); // 归一到255
    nh.param("bev/use_dense", use_dense_, true);               // 是否使用密度图

    x_min_ind_ = static_cast<int>(std::floor(min_x_ / resolution_));
    x_max_ind_ = static_cast<int>(std::floor(max_x_ / resolution_));
    y_min_ind_ = static_cast<int>(std::floor(min_y_ / resolution_));
    y_max_ind_ = static_cast<int>(std::floor(max_y_ / resolution_));
    x_num_ = x_max_ind_ - x_min_ind_ + 1;
    y_num_ = y_max_ind_ - y_min_ind_ + 1;
}

void BEVProjector::voxelDownSample(BEVFrame& frame) {
    if (!frame.points || frame.points->empty()) {
        std::cout << "Point cloud is empty, skipping voxel downsampling." << std::endl;
        return;
    }
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(frame.points);
    sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    sor.filter(*frame.points);
}

V3D BEVProjector::turn_pixel_to_point(const V3D& pixel_uv) {
    float x = (x_max_ind_ - pixel_uv.x()) * resolution_;
    float y = (y_max_ind_ - pixel_uv.y()) * resolution_; 
    return V3D(y, x, 0.0);
}

void BEVProjector::getBEV(BEVFrame& frame) {
    if (downsample_) {
        voxelDownSample(frame);
    }

    cv::Mat mat_global_image = cv::Mat::zeros(y_num_, x_num_, CV_8UC1);
    frame.img_dense = cv::Mat::zeros(y_num_, x_num_, CV_32FC1);

    if (!frame.points) {
        std::cout << "Point cloud is empty, skipping BEV projection." << std::endl;
        return;   
    }

    for (size_t i = 0; i < frame.points->points.size(); ++i) {
        const auto& point = frame.points->points[i];
        float x = point.x;
        float y = point.y;

        float x_float = (y / resolution_);
        float y_float = (x / resolution_);
        int x_ind = x_max_ind_ - static_cast<int>(std::floor(x_float));
        int y_ind = y_max_ind_ - static_cast<int>(std::floor(y_float));

        float x_frac = x_float - std::floor(x_float);
        float y_frac = y_float - std::floor(y_float);

        if (x_ind >= x_num_ || y_ind >= y_num_ || x_ind < 0 || y_ind < 0) {
            continue;
        }

        if (mat_global_image.at<uchar>(y_ind, x_ind) < 15) {
            mat_global_image.at<uchar>(y_ind, x_ind) += 1;
        }
    }

    if(normalize_to_255_){
        mat_global_image.setTo(0, mat_global_image <= 1);
        mat_global_image *= 15;
        mat_global_image.setTo(255, mat_global_image > 255);
    }

    frame.img_dense = mat_global_image.clone();
}
