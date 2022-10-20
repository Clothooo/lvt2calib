#ifndef excalib_min2d_H
#define excalib_min2d_H

#include <ceres/ceres.h>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;


class excalib_min2d
{
private:
    pcl::PointXYZ p_l_3d_;
    cv::Point2f p_c_2d_;
    Eigen::Matrix3d cameraMatrix_;

public:
    excalib_min2d(pcl::PointXYZ p_l_3d, cv::Point2f p_c_2d, Eigen::Matrix3d cameraMatrix);
    excalib_min2d(){};
    ~excalib_min2d(){};
    Eigen::Matrix3d get_camMatrix()
    {
        return cameraMatrix_;
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals) const {
        Eigen::Matrix<T, 3, 3> innerT = cameraMatrix_.cast<T>();
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> p_l(T(p_l_3d_.x), T(p_l_3d_.y), T(p_l_3d_.z));
        Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
         
        Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;

        residuals[0] = p_2[0]/p_2[2] - T(p_c_2d_.x);
        residuals[1] = p_2[1]/p_2[2] - T(p_c_2d_.y);

        return true;
    }

    static ceres::CostFunction *Create(pcl::PointXYZ p_l_3d, cv::Point2f p_c_2d, Eigen::Matrix3d cameraMatrix) {
        return (new ceres::AutoDiffCostFunction<excalib_min2d, 2, 4, 3>(new excalib_min2d(p_l_3d, p_c_2d, cameraMatrix)));
    }
};

excalib_min2d::excalib_min2d(pcl::PointXYZ p_l_3d, cv::Point2f p_c_2d, Eigen::Matrix3d cameraMatrix) {
    p_l_3d_ = p_l_3d;
    p_c_2d_ = p_c_2d;
    cameraMatrix_ = cameraMatrix;
}


#endif