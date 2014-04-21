
#ifndef WORLD2CAM_H
#define WORLD2CAM_H

#include <Eigen/Core>
#include <opencv2/core.hpp>

void myworld2cam(cv::Vec2d &point2D, const Eigen::Vector3d &point3D, struct ocam_model *myocam_model);

#endif