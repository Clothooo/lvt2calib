#ifndef ouster_utils_H
#define ouster_utils_H

#define PCL_NO_PRECOMPILE
#define DEBUG 0

#include <vector>
#include <cmath>
#include <math.h>   // by jun
#include <unistd.h>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/impl/convolution_3d.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/feature.hpp>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>

// #include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <opencv2/core/core.hpp>

#ifdef TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#else
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#endif


using namespace std;
using namespace cv;
// using namespace sensor_msgs;

typedef Eigen::Matrix<double, 12, 1> Vector12d;
enum OUSTER_TYPE {OUSTER_32, OUSTER_64, OUSTER_128};

static const int RINGS_COUNT_OUSTER32 = 32;
static const int RINGS_COUNT_OUSTER64 = 64;
static const int RINGS_COUNT_OUSTER128 = 128;
vector<int> rings_count_v = {
  RINGS_COUNT_OUSTER32,
  RINGS_COUNT_OUSTER64,
  RINGS_COUNT_OUSTER128
};
OUSTER_TYPE laser_type = OUSTER_32;

namespace Ouster {
  struct Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }EIGEN_ALIGN16;

  void addRange(pcl::PointCloud<Ouster::Point> & pc){
    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
    }
  }

  vector<vector<Point*> > getRings(pcl::PointCloud<Ouster::Point> & pc, OUSTER_TYPE type_ = OUSTER_32)
  {
    vector<vector<Point*> > rings;
    rings.resize(rings_count_v[type_]);
    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      rings[pt->ring].push_back(&(*pt));
    }
    return rings;
  }

  void resetIntensity(pcl::PointCloud<Ouster::Point> & pc)
  {
    vector<vector<Ouster::Point*> > rings = Ouster::getRings(pc, laser_type);
    for (vector<vector<Ouster::Point*> >::iterator ring = rings.begin(); ring < rings.end(); ++ring){
      if (ring->empty()) continue;

      (*ring->begin())->intensity = 0;
      (*(ring->end() - 1))->intensity = 0;
      for (vector<Ouster::Point*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++){
        Ouster::Point *prev = *(pt - 1);
        Ouster::Point *succ = *(pt + 1);
        (*pt)->intensity = max( max( prev->range-(*pt)->range, succ->range-(*pt)->range), 0.f);
      }
    }
  }

  // all intensities to range min-max
  void normalizeIntensity(pcl::PointCloud<Point> & pc, float minv, float maxv)
  {
    float min_found = INFINITY;
    float max_found = -INFINITY;

    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      max_found = max(max_found, pt->intensity);
      min_found = min(min_found, pt->intensity);
    }

    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (maxv - minv) + minv;
    }
  }

  void copyReflectivityToIntensity(pcl::PointCloud<Point> & pc)
  {
    for(auto pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      pt->intensity = pt->reflectivity;
    }
  }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Ouster::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (float, range, range)
)
// PCL_INSTANTIATE(PCLBase, Ouster::Point);
// PCL_INSTANTIATE(Convolution3D, Ouster::Point);
// PCL_INSTANTIATE(GaussianKernel, Ouster::Point);
// PCL_INSTANTIATE(KdTree, Ouster::Point);
// PCL_INSTANTIATE(KdTreeFLANN, Ouster::Point);
// PCL_INSTANTIATE(RegionGrowing, Ouster::Point);

void findLaserType(int laser_ring_num)
{
  auto it = find(rings_count_v.begin(), rings_count_v.end(), laser_ring_num);
  if(it != rings_count_v.end())
  {
    int idx = distance(rings_count_v.begin(), it);
    switch (idx)
    {
    case 0:
      laser_type = OUSTER_32;
      cout << "laser_type: OUSTER_32" << endl;
      break;
    case 1:
      laser_type = OUSTER_64;
      cout << "laser_type: OUSTER_64" << endl;
      break;
    case 2:
      laser_type = OUSTER_128;
      cout << "laser_type: OUSTER_128" << endl;
      break;

    default:
      break;
    }
  }
  else
  {
    laser_type = OUSTER_32;
  }
  
  return;
}

Eigen::Affine3f getRotationMatrix(Eigen::Vector3f source, Eigen::Vector3f target){
  Eigen::Vector3f rotation_vector = target.cross(source);
  rotation_vector.normalize();
  double theta = acos(source[2]/sqrt( pow(source[0],2)+ pow(source[1],2) + pow(source[2],2)));

  if(DEBUG) ROS_INFO("Rot. vector: (%lf %lf %lf) / Angle: %lf", rotation_vector[0], rotation_vector[1], rotation_vector[2], theta);

  Eigen::Matrix3f rotation = Eigen::AngleAxis<float>(theta, rotation_vector) * Eigen::Scaling(1.0f);
  Eigen::Affine3f rot(rotation);
  return rot;
}

void getCenterClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud,
  double cluster_tolerance = 0.10, int min_cluster_size = 15, int max_cluster_size = 200, bool verbosity=true){

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
  euclidean_cluster.setClusterTolerance (cluster_tolerance);
  euclidean_cluster.setMinClusterSize (min_cluster_size);
  euclidean_cluster.setMaxClusterSize (max_cluster_size);
  euclidean_cluster.setSearchMethod (tree);
  euclidean_cluster.setInputCloud (cloud_in);
  euclidean_cluster.extract (cluster_indices);

  if(DEBUG && verbosity) cout << cluster_indices.size() << " clusters found from " << cloud_in->points.size() << " points in cloud" << endl;

  for (std::vector<pcl::PointIndices>::iterator it=cluster_indices.begin(); it<cluster_indices.end(); it++) {
    float accx = 0., accy = 0., accz = 0.;
    for(vector<int>::iterator it2=it->indices.begin(); it2<it->indices.end(); it2++){
      accx+=cloud_in->at(*it2).x;
      accy+=cloud_in->at(*it2).y;
      accz+=cloud_in->at(*it2).z;
    }
    // Compute and add center to clouds
    pcl::PointXYZ center;
    center.x =  accx/it->indices.size();
    center.y =  accy/it->indices.size();
    center.z =  accz/it->indices.size();
    centers_cloud->push_back(center);
  }
}

#endif
