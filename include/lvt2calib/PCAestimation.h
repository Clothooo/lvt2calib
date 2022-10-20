#define PCL_NO_PRECOMPILE
#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;

void ComputeEigenVectorPCA(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Vector4f& pcaCentroid, Eigen::Matrix3f& eigenVectorsPCA, Eigen::Vector3f& eigenValuesPCA)
{
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenValuesPCA = eigen_solver.eigenvalues();

    return;
}
