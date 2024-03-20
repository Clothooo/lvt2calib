/*
  lvt2calib - Automatic calibration algorithm for extrinsic parameters of a stereo camera and a velodyne
  Copyright (C) 2017-2018 Jorge Beltran, Carlos Guindel
  This file is part of lvt2calib.
  lvt2calib is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.
  lvt2calib is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with lvt2calib.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  laser_pattern: Find the circle centers in the laser cloud
*/

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <dynamic_reconfigure/server.h>

#include <lvt2calib/VeloCircleConfig.h>
#include <lvt2calib/velo_utils.h>
#include <lvt2calib/ClusterCentroids.h>

using namespace std;
using namespace sensor_msgs;
using namespace pcl;

#define DEBUG 0

typedef Velodyne::Point PointType;
typedef pcl::PointCloud<PointType> CloudType;

ros::Publisher cumulative_pub, centers_pub, circle_center_pub, centers_centroid_pub, pattern_pub, range_pub, edges_pub, pattern_plane_edges_pub, coeff_pub, aux_pub, auxpoint_pub, debug_pub;
int nFrames; // Used for resetting center computation
pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;   // Accumulated centers

// Dynamic parameters
double edge_depth_thre__;
double passthrough_radius_min_, passthrough_radius_max_, circle_radius_,
       centroid_distance_min_, centroid_distance_max_;
Eigen::Vector3f axis_;
double angle_threshold_;
double cluster_size_;
double edge_depth_thre_, edge_knn_radius_;
int clouds_proc_ = 0, clouds_used_ = 0;
int min_centers_found_;
int rings_count;

string ns_str;

void callback(const PointCloud2::ConstPtr& laser_cloud, const PointCloud2::ConstPtr& calib_cloud)
{

  if(DEBUG) ROS_INFO("[%s] Processing cloud...", ns_str.c_str());

  CloudType::Ptr velo_cloud_pc (new CloudType),
                                        calib_board_pc(new CloudType),
                                        pattern_cloud(new CloudType);

  clouds_proc_++;

  fromROSMsg(*laser_cloud, *velo_cloud_pc);
  fromROSMsg(*calib_cloud, *calib_board_pc);
  sensor_msgs::PointCloud2 range_ros;
  pcl::toROSMsg(*calib_board_pc, range_ros);
  range_ros.header = laser_cloud->header;
  range_pub.publish(range_ros);   // topic: /laser_pattern/range_filtered_velo

  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentation<PointType> plane_segmentation;
  plane_segmentation.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  plane_segmentation.setDistanceThreshold (0.01);
  plane_segmentation.setMethodType (pcl::SAC_RANSAC);
  plane_segmentation.setAxis(Eigen::Vector3f(axis_[0], axis_[1], axis_[2]));
  plane_segmentation.setEpsAngle (angle_threshold_);
  plane_segmentation.setOptimizeCoefficients (true);
  plane_segmentation.setMaxIterations(1000);
  plane_segmentation.setInputCloud (calib_board_pc);
  plane_segmentation.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_WARN("[%s] Could not estimate a planar model for the given dataset.", ns_str.c_str());
    return;
  }

  // Copy coefficients to proper object for further filtering
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = coefficients->values[0];
  coefficients_v(1) = coefficients->values[1];
  coefficients_v(2) = coefficients->values[2];
  coefficients_v(3) = coefficients->values[3];

  // Get edges points by range
  // vector<vector<PointType*> > rings = Velodyne::getRings(*velo_cloud_pc, laser_type);
  // for (vector<vector<PointType*> >::iterator ring = rings.begin(); ring < rings.end(); ++ring){
  //   if (ring->empty()) continue;

  //   (*ring->begin())->intensity = 0;
  //   (*(ring->end() - 1))->intensity = 0;
  //   for (vector<PointType*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++){
  //     PointType *prev = *(pt - 1);
  //     PointType *succ = *(pt + 1);
  //     (*pt)->intensity = max( max( prev->range-(*pt)->range, succ->range-(*pt)->range), 0.f);
  //   }
  // }

  vector<int> indices_f1, indices_f2;
  pcl::removeNaNFromPointCloud(*velo_cloud_pc, *velo_cloud_pc, indices_f1);
  pcl::removeNaNFromPointCloud(*calib_board_pc, *calib_board_pc, indices_f2);
  CloudType::Ptr edges_cloud(new CloudType);
  pcl::PointCloud<pcl::PointXYZ>::Ptr calib_board_pc_copy(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*calib_board_pc, *calib_board_pc_copy);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(calib_board_pc_copy);

  for (CloudType::iterator pt = velo_cloud_pc->points.begin(); pt < velo_cloud_pc->points.end(); ++pt)
  {
    vector<int> pointIdxNKNSearch;
    vector<float> pointNKNSquaredDistance;
    pcl::PointXYZ searchP;
    pcl::copyPoint(*pt, searchP);
    if(kdtree.nearestKSearch(searchP, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      // cout << "pointNKNSquaredDistance: " << pointNKNSquaredDistance[0] << endl;
      if(pointNKNSquaredDistance[0] <= edge_knn_radius_)
      {
        if(pt->intensity>edge_depth_thre_){
          edges_cloud->push_back(*pt);
        }
      }
    }
  }

  if (edges_cloud->points.size () == 0)
  {
    ROS_WARN("[%s] Could not detect pattern edges.", ns_str.c_str());
    return;
  }

  // Get points belonging to plane in pattern pointcloud
  pcl::SampleConsensusModelPlane<PointType>::Ptr dit (new pcl::SampleConsensusModelPlane<PointType> (edges_cloud));
  std::vector<int> inliers2;
  dit -> selectWithinDistance (coefficients_v, .05, inliers2); // 0.1
  pcl::copyPointCloud<PointType>(*edges_cloud, inliers2, *pattern_cloud);

  sensor_msgs::PointCloud2 edges_ros;
  pcl::toROSMsg(*edges_cloud, edges_ros);
  edges_ros.header = laser_cloud->header;
  edges_pub.publish(edges_ros);   // topic: /laser_pattern/edges_cloud

  sensor_msgs::PointCloud2 plane_edges_cloud_ros;
  pcl::toROSMsg(*pattern_cloud, plane_edges_cloud_ros);
  plane_edges_cloud_ros.header = laser_cloud->header;
  pattern_plane_edges_pub.publish(plane_edges_cloud_ros);   // topic: /laser_pattern/plane_edges_cloud
  

  // Remove kps not belonging to circles by coords
  pcl::PointCloud<pcl::PointXYZ>::Ptr circles_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  vector<vector<PointType*> > rings2 = Velodyne::getRings(*pattern_cloud, laser_type);
  int ringsWithCircle = 0;
  for (vector<vector<PointType*> >::iterator ring = rings2.begin(); ring < rings2.end(); ++ring){
    if(ring->size() < 4){
      ring->clear();
    }else{ // Remove first and last points in ring
      ringsWithCircle++;
      ring->erase(ring->begin());
      ring->pop_back();

      for (vector<PointType*>::iterator pt = ring->begin(); pt < ring->end(); ++pt){
        // Velodyne specific info no longer needed for calibration
        // so standard point is used from now on
        pcl::PointXYZ point;
        point.x = (*pt)->x;
        point.y = (*pt)->y;
        point.z = (*pt)->z;
        circles_cloud->push_back(point);
      }
    }
  }

  if(circles_cloud->points.size() > ringsWithCircle*4){
    ROS_WARN("[%s] Too many outliers, not computing circles.", ns_str.c_str());
    return;
  }

  sensor_msgs::PointCloud2 velocloud_ros2;
  pcl::toROSMsg(*circles_cloud, velocloud_ros2);
  velocloud_ros2.header = laser_cloud->header;
  pattern_pub.publish(velocloud_ros2);   // topic: /laser_pattern/pattern_circles

  // Show circles cloud
  if(DEBUG) 
    for(int i=0; i<circles_cloud->points.size(); i++){
      // 
      cout  << "Circle points " << i << ": " << circles_cloud->points[i].x << " " << circles_cloud->points[i].y << " " << circles_cloud->points[i].z << "\n";
    }

  // Rotate cloud to face pattern plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3f xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;
  // ???
  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  Eigen::Affine3f rotation = getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*circles_cloud, *xy_cloud, rotation);

  // This aux_point (0, 0, -d/c) is on the plane (ax+by+cz+d=0)
  pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3)/coefficients_v(2));
  aux_cloud->push_back(aux_point);   

  pcl::PointCloud<pcl::PointXYZ>::Ptr auxrotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);

  sensor_msgs::PointCloud2 ros_auxpoint;
  pcl::toROSMsg(*auxrotated_cloud, ros_auxpoint);
  ros_auxpoint.header = laser_cloud->header;
  auxpoint_pub.publish(ros_auxpoint);   // topic: /laser_pattern/rotated_pattern

  double zcoord_xyplane = auxrotated_cloud->at(0).z;
  if(DEBUG) cout << "zcoord_xyplane = " << zcoord_xyplane << endl;

  pcl::PointXYZ edges_centroid;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (xy_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
  euclidean_cluster.setClusterTolerance (0.55);
  euclidean_cluster.setMinClusterSize (12);
  euclidean_cluster.setMaxClusterSize (rings_count_v[laser_type]*4);
  euclidean_cluster.setSearchMethod (tree);
  euclidean_cluster.setInputCloud (xy_cloud);
  euclidean_cluster.extract (cluster_indices);

  if(DEBUG) cout << cluster_indices.size() << " clusters found from " << xy_cloud->points.size() << " points in cloud" << endl;

  for (std::vector<pcl::PointIndices>::iterator it=cluster_indices.begin(); it<cluster_indices.end(); ++it) {
    float accx = 0., accy = 0., accz = 0.;
    for(vector<int>::iterator it2=it->indices.begin(); it2<it->indices.end(); ++it2){
      accx+=xy_cloud->at(*it2).x;
      accy+=xy_cloud->at(*it2).y;
      accz+=xy_cloud->at(*it2).z;
    }
    // Compute and add center to clouds
    edges_centroid.x =  accx/it->indices.size();
    edges_centroid.y =  accy/it->indices.size();
    edges_centroid.z =  accz/it->indices.size();
    if(DEBUG) ROS_INFO("Centroid %f %f %f", edges_centroid.x, edges_centroid.y, edges_centroid.z);
  }

  // Extract circles
  pcl::ModelCoefficients::Ptr coefficients3 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers3 (new pcl::PointIndices);

  // Ransac settings for circle detection
  pcl::SACSegmentation<pcl::PointXYZ> circle_segmentation;
  circle_segmentation.setModelType (pcl::SACMODEL_CIRCLE2D);
  circle_segmentation.setDistanceThreshold (0.04);
  circle_segmentation.setMethodType (pcl::SAC_RANSAC);
  circle_segmentation.setOptimizeCoefficients (true);
  circle_segmentation.setMaxIterations(1000);
  circle_segmentation.setRadiusLimits(circle_radius_- 0.02, circle_radius_+ 0.02);

  pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZ>); // Used for removing inliers
  pcl::copyPointCloud<pcl::PointXYZ>(*xy_cloud, *copy_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>); // To store circle points
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>); // To store circle points
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>); // Temp pc used for swaping

  // Force pattern points to belong to computed plane
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = copy_cloud->points.begin(); pt < copy_cloud->points.end(); ++pt){
    pt->z = zcoord_xyplane;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  std::vector< std::vector<float> > found_centers;
  std::vector<pcl::PointXYZ> centroid_cloud_inliers;
  bool valid = true;   // if it is a valid center 

  while ((copy_cloud->points.size()+centroid_cloud_inliers.size()) > 3 && found_centers.size()<4 && copy_cloud->points.size()){
    circle_segmentation.setInputCloud (copy_cloud);
    circle_segmentation.segment (*inliers3, *coefficients3);
    if (inliers3->indices.size () == 0)
    {
      break;
    }

    // Extract the inliers
    extract.setInputCloud (copy_cloud);
    extract.setIndices (inliers3);
    extract.setNegative (false);
    extract.filter (*circle_cloud);

    sensor_msgs::PointCloud2 range_ros2;
    pcl::toROSMsg(*circle_cloud, range_ros2);
    range_ros2.header = laser_cloud->header;
    debug_pub.publish(range_ros2);   // topic: /laser_pattern/debug

    // Add center point to cloud
    pcl::PointXYZ center;
    center.x = *coefficients3->values.begin();
    center.y = *(coefficients3->values.begin()+1);
    center.z = zcoord_xyplane;
    // Make sure there is no circle at the center of the pattern or far away from it
    double centroid_distance = sqrt(pow(fabs(edges_centroid.x-center.x),2) + pow(fabs(edges_centroid.y-center.y),2));
    if(DEBUG) ROS_INFO("Distance to centroid %f, should be in (%.2f, %.2f)", centroid_distance, centroid_distance_min_, centroid_distance_max_);
    if (centroid_distance < centroid_distance_min_){
      valid = false;
      // ???
      for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = circle_cloud->points.begin(); pt < circle_cloud->points.end(); ++pt){
        centroid_cloud_inliers.push_back(*pt);
      }
    }else if(centroid_distance > centroid_distance_max_){
      valid = false;
    }else{
      if(DEBUG) ROS_INFO("Valid centroid");
      for(std::vector<std::vector <float> >::iterator it = found_centers.begin(); it != found_centers.end(); ++it) {
        if(DEBUG) ROS_INFO("%f", sqrt(pow(fabs((*it)[0]-center.x),2) + pow(fabs((*it)[1]-center.y),2)));
        if (sqrt(pow(fabs((*it)[0]-center.x),2) + pow(fabs((*it)[1]-center.y),2))<0.25){
          valid = false;
          break;
        }
      }

      // If center is valid, check if any point from wrong_circle belongs to it, and pop it if true
      for (std::vector<pcl::PointXYZ>::iterator pt = centroid_cloud_inliers.begin(); pt < centroid_cloud_inliers.end(); ++pt){
        if(DEBUG) cout << "In schrodinger_pt" << endl;
        pcl::PointXYZ schrodinger_pt((*pt).x, (*pt).y, (*pt).z);
        double distance_to_cluster = sqrt(pow(schrodinger_pt.x-center.x,2) + pow(schrodinger_pt.y-center.y,2) + pow(schrodinger_pt.z-center.z,2));
        // if(DEBUG) ROS_INFO("Distance to cluster: %lf", distance_to_cluster);
        if(distance_to_cluster<circle_radius_+0.02){
          centroid_cloud_inliers.erase(pt);
          --pt; // To avoid out of range
        }
      }
      // if(DEBUG) ROS_INFO("Remaining inliers %lu", centroid_cloud_inliers.size());
    }

    if (valid){
      // if(DEBUG) ROS_INFO("Valid circle found");
      std::vector<float> found_center;
      found_center.push_back(center.x);
      found_center.push_back(center.y);
      found_center.push_back(center.z);
      found_centers.push_back(found_center);
      // if(DEBUG) ROS_INFO("Remaining points in cloud %lu", copy_cloud->points.size());
    }

    // Remove inliers from pattern cloud to find next circle
    extract.setNegative (true);
    extract.filter(*cloud_f);
    copy_cloud.swap(cloud_f);
    valid = true;

    if(DEBUG) ROS_INFO("Remaining points in cloud %lu", copy_cloud->points.size());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);   // One frame of centers

  if(found_centers.size() >= min_centers_found_ && found_centers.size() < 5){
    for (std::vector<std::vector<float> >::iterator it = found_centers.begin(); it < found_centers.end(); ++it){
      pcl::PointXYZ center;
      center.x = (*it)[0];
      center.y = (*it)[1];
      center.z = (*it)[2];
      pcl::PointXYZ center_rotated_back = pcl::transformPoint(center, rotation.inverse());
      center_rotated_back.x = (- coefficients->values[1] * center_rotated_back.y - coefficients->values[2] * center_rotated_back.z - coefficients->values[3])/coefficients->values[0];
      cumulative_cloud->push_back(center_rotated_back);
      circle_center_cloud->push_back(center_rotated_back);
    }

    sensor_msgs::PointCloud2 ros_pointcloud;
    pcl::toROSMsg(*cumulative_cloud, ros_pointcloud);
    ros_pointcloud.header = laser_cloud->header;
    cumulative_pub.publish(ros_pointcloud);   // Topic: /laser_pattern/cumulative_cloud

    sensor_msgs::PointCloud2 ros_circle_center_cloud;
    pcl::toROSMsg(*circle_center_cloud, ros_circle_center_cloud);
    ros_circle_center_cloud.header = laser_cloud->header;
    circle_center_pub.publish(ros_circle_center_cloud);   // Topic: /laser_pattern/circle_center_cloud

  }else{
    ROS_WARN("[%s] Not enough centers: %ld", ns_str.c_str(), found_centers.size());
    return;
  }

  circle_cloud.reset();
  copy_cloud.reset(); // Free memory
  cloud_f.reset(); // Free memory

  nFrames++;
  clouds_used_ = nFrames;

  pcl_msgs::ModelCoefficients m_coeff;
  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = laser_cloud->header;
  coeff_pub.publish(m_coeff);  // Topic : /laser_pattern/plane_model

  ROS_INFO("[%s] %d/%d frames: %ld pts in cloud", ns_str.c_str(), clouds_used_, clouds_proc_, cumulative_cloud->points.size());

  // Create cloud for publishing centers
  pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Compute circles centers
  // Publish cumulative center cloud
  getCenterClusters(cumulative_cloud, centers_cloud, cluster_size_, nFrames/2, nFrames);
  if (centers_cloud->points.size()>4){
    getCenterClusters(cumulative_cloud, centers_cloud, cluster_size_, 3.0*nFrames/4.0, nFrames);
  }

  if (centers_cloud->points.size()==4){

    // sensor_msgs::PointCloud2 ros2_pointcloud;
    // pcl::toROSMsg(*centers_cloud, ros2_pointcloud);
    // ros2_pointcloud.header = laser_cloud->header;

    // // Cumulated center cloud centroid
    // lvt2calib::ClusterCentroids to_send;
    // to_send.header = laser_cloud->header;
    // to_send.cluster_iterations = clouds_used_;
    // to_send.total_iterations = clouds_proc_;
    // to_send.cloud = ros2_pointcloud;

    sensor_msgs::PointCloud2 ros2_centers_centroid_cloud;
    pcl::toROSMsg(*centers_cloud, ros2_centers_centroid_cloud);
    ros2_centers_centroid_cloud.header = laser_cloud->header;
    centers_centroid_pub.publish(ros2_centers_centroid_cloud);   // Topic: /laser_pattern/centers_centroid_cloud


    sensor_msgs::PointCloud2 ros2_pointcloud;
    pcl::toROSMsg(*circle_center_cloud, ros2_pointcloud);
    ros2_pointcloud.header = laser_cloud->header;

    // Center of one scan
    lvt2calib::ClusterCentroids to_send;
    to_send.header = laser_cloud->header;
    to_send.cluster_iterations = clouds_used_;
    to_send.total_iterations = clouds_proc_;
    to_send.cloud = ros2_pointcloud;

    centers_pub.publish(to_send);   // Topic: /laser_pattern/centers_cloud
    ROS_INFO("Pattern centers published");
  }
}

void param_callback(lvt2calib::VeloCircleConfig &config, uint32_t level){
  circle_radius_ = config.circle_radius;
  ROS_INFO("New pattern circle radius: %f", circle_radius_);
  axis_[0] = config.x;
  axis_[1] = config.y;
  axis_[2] = config.z;
  ROS_INFO("New normal axis for plane segmentation: %f, %f, %f", axis_[0], axis_[1], axis_[2]);
  angle_threshold_ = config.angle_threshold;
  ROS_INFO("New angle angle_threshold: %f", angle_threshold_);
  edge_depth_thre_ = config.edge_depth_thre;
  ROS_INFO("New edge_depth_thre: %f", edge_depth_thre_);
  edge_knn_radius_ = config.edge_knn_radius;
  ROS_INFO("New edge_knn_radius: %f", edge_knn_radius_);
  centroid_distance_min_ = config.centroid_distance_min;
  ROS_INFO("New minimum distance between centroids: %f", centroid_distance_min_);
  centroid_distance_max_ = config.centroid_distance_max;
  ROS_INFO("New maximum distance between centroids: %f", centroid_distance_max_);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "velo_pattern_circle");
  ros::NodeHandle nh_("~"); // LOCAL
  // ros::Subscriber sub = nh_.subscribe ("cloud1", 1, callback);

  nh_.param("cluster_size", cluster_size_, 0.02);
  nh_.param("min_centers_found", min_centers_found_, 4);
  nh_.param<std::string>("ns", ns_str, "laser");
  nh_.param("laser_ring_num", rings_count, 16);
  findLaserType(rings_count);
  // switch (rings_count)
  // {
  // case 16:
  //   laser_type = VELO_16;
  //   ROS_INFO("LASER_TYPE: VELO_16");
  //   break;
  // case 32:
  //   laser_type = VELO_32;
  //   ROS_INFO("LASER_TYPE: VELO_32");
  //   break;
  // case 64:
  //   laser_type = VELO_64;
  //   ROS_INFO("LASER_TYPE: VELO_64");
  //   break;
  // case 128:
  //   laser_type = VELO_128;  
  //   ROS_INFO("LASER_TYPE: VELO_128");
  //   break;
  // default:
  //   ROS_WARN("Invalid 'laser_ring_num'!!!");
  //   break;
  // }

  range_pub = nh_.advertise<PointCloud2> ("range_filtered_velo", 1);
  edges_pub = nh_.advertise<PointCloud2> ("edges_cloud", 1);
  pattern_plane_edges_pub = nh_.advertise<PointCloud2> ("plane_edges_cloud", 1);


  pattern_pub = nh_.advertise<PointCloud2> ("pattern_circles", 1);
  auxpoint_pub = nh_.advertise<PointCloud2> ("rotated_pattern", 1);
  cumulative_pub = nh_.advertise<PointCloud2> ("cumulative_cloud", 1);
  centers_pub = nh_.advertise<lvt2calib::ClusterCentroids> ("/"+ns_str+"/centers_cloud", 1);
  circle_center_pub = nh_.advertise<PointCloud2> ("circle_center_cloud", 1);
  centers_centroid_pub= nh_.advertise<PointCloud2> ("centers_centroid_cloud", 1);

  debug_pub = nh_.advertise<PointCloud2> ("debug", 1);

  coeff_pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("plane_model", 1);


  nFrames = 0;
  cumulative_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  dynamic_reconfigure::Server<lvt2calib::VeloCircleConfig> server;
  dynamic_reconfigure::Server<lvt2calib::VeloCircleConfig>::CallbackType f;
  f = boost::bind(param_callback, _1, _2);
  server.setCallback(f);

  message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub(nh_, "laser_cloud", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> calib_sub(nh_, "calib_cloud", 10);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, calib_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::Rate loop_rate(10);
  bool pause_process = false;
  bool end_process = false;
  bool do_acc_boards = false;
  while(ros::ok())
  {
    // ros::param::get("/pause_process", pause_process);
    ros::param::get("/end_process", end_process);
    ros::param::get("/do_acc_boards", do_acc_boards);
    if(end_process)
    {
      ROS_WARN("[%s/velo_pattern_circle] END......", ns_str.c_str());
      break;
    }
    if(!do_acc_boards)
    {
      ROS_WARN("[%s/velo_pattern_circle] PAUSED......", ns_str.c_str());
      while (!do_acc_boards && ros::ok())
      {
        // ros::param::get("/pause_process", pause_process);
        ros::param::get("/end_process", end_process);
        ros::param::get("/do_acc_boards", do_acc_boards);
        if(end_process)
        {
          ROS_WARN("[%s/velo_pattern_circle] END......", ns_str.c_str());
          break;
        }
      }
      if(end_process)
        break;
      clouds_proc_ = 0;
      clouds_used_ = 0;
      nFrames = 0;
      cumulative_cloud->clear();
    }
    ros::spinOnce();
  }

  ros::shutdown();
  return 0;
}