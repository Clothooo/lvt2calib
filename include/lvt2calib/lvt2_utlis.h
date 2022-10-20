#ifndef lvt2_utlis_H
#define lvt2_utlis_H

#define PCL_NO_RECOMPILE
#define DEBUG 0

#include <vector>
#include <string>
#include <cmath>
#include <math.h>   
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

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
#endif

using namespace std;
using namespace cv;
using namespace pcl;
using namespace tf;

void sortPatternCentersYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<pcl::PointXYZ> &v){
  double avg_y = 0, avg_z = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    avg_y += (*it).y;
    avg_z += (*it).z;
  }

  pcl::PointXYZ center;
  center.y = avg_y/4.;
  center.z = avg_z/4.;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    double y_dif = (*it).y - center.y;
    double z_dif = (*it).z - center.z;

    if(y_dif > 0 && z_dif > 0){
      v[0] = (*it);
    }else if(y_dif < 0 && z_dif > 0){
      v[1] = (*it);
    }else if(y_dif < 0 && z_dif < 0){
      v[2] = (*it);
    }else{
      v[3] = (*it);
    }
  }
}

pcl::PointXYZ calculateClusterCentroid(std::vector<pcl::PointXYZ> one_acc_points){
  // calculate the centroid of an accumulated points
  double avg_x = 0, avg_y = 0, avg_z = 0;
  for(std::vector<pcl::PointXYZ>::iterator it=one_acc_points.begin(); it<one_acc_points.end(); it++){
    avg_x += (*it).x;
    avg_y += (*it).y;
    avg_z += (*it).z;
  }

  pcl::PointXYZ centroid;
  int number_of_points = one_acc_points.size();
  centroid.x = avg_x/double(number_of_points);
  centroid.y = avg_y/double(number_of_points);
  centroid.z = avg_z/double(number_of_points);

  return centroid;
}

cv::Point2f calculateClusterCentroid2d(std::vector<cv::Point2f> one_acc_points)
{
  double avg_x = 0, avg_y = 0, avg_z = 0;
  for(auto it=one_acc_points.begin(); it<one_acc_points.end(); it++){
    avg_x += (*it).x;
    avg_y += (*it).y;
  }

  cv::Point2f centroid;
  int number_of_points = one_acc_points.size();
  // cout << "number_of_points = " << number_of_points << endl;
  centroid.x = avg_x/double(number_of_points);
  centroid.y = avg_y/double(number_of_points);
  
  return centroid;

}

std::vector<double> calculateRMSE(std::vector<pcl::PointXYZ> ground_truth, std::vector<pcl::PointXYZ> detected){
  int N = ground_truth.size();
  double sum = 0, sum_x = 0, sum_y = 0, sum_z = 0;
  double rmse_total = 0,rmse_x = 0, rmse_y = 0, rmse_z = 0;

  for (int i=0; i < N; i++){
    sum_x += pow(detected[i].x - ground_truth[i].x, 2);
    sum_y += pow(detected[i].y - ground_truth[i].y, 2);
    sum_z += pow(detected[i].z - ground_truth[i].z, 2);
  }

  sum = sum_x + sum_y + sum_z;

  sum_x = sum_x / double(N);
  sum_y = sum_y / double(N);
  sum_z = sum_z / double(N);
  sum = sum / double(N);

  rmse_x = sqrt(sum_x);
  rmse_y = sqrt(sum_y);
  rmse_z = sqrt(sum_z);
  rmse_total = sqrt(sum);
  
  // cout << "rmse_total 1 = " << rmse_total << endl;
  // cout << "rmse_total 2 = " << sqrt(pow(rmse_x, 2) + pow(rmse_y, 2) + pow(rmse_z, 2)) << endl;

  std::vector<double> rmse = {rmse_x, rmse_y, rmse_z, rmse_total};
  // pcl::PointXYZ rmse_p;
  // rmse_p.x = rmse_x;
  // rmse_p.y = rmse_y;
  // rmse_p.z = rmse_z;

  return rmse;
}

std::vector<double> calculateRMSE(std::vector<cv::Point2f> ground_truth, std::vector<cv::Point2f> detected){
  int N = ground_truth.size();
  if(DEBUG)
  {
    cout << "size of ground_truth: "<< N << endl;
    cout << "size of detected: " << detected.size() << endl;
  }
  double sum = 0, sum_x = 0, sum_y = 0;
  double rmse_total = 0, rmse_x = 0, rmse_y = 0;

  for (int i=0; i < N; i++){
    sum_x += pow(detected[i].x - ground_truth[i].x, 2);
    sum_y += pow(detected[i].y - ground_truth[i].y, 2);
  }

  sum_x = sum_x / double(N);
  sum_y = sum_y / double(N);

  rmse_x = sqrt(sum_x);
  rmse_y = sqrt(sum_y);
  rmse_total = sqrt(pow(rmse_x, 2) + pow(rmse_y, 2));

  cv::Point2f rmse_p = cv::Point2f(rmse_x, rmse_y);
  // rmse_p.x = rmse_x;
  // rmse_p.y = rmse_y;

  std::vector<double> rmse = {rmse_x, rmse_y, rmse_total};
  return rmse;
}

void sortPatternCentersUV(std::vector<cv::Point2f> p, std::vector<cv::Point2f> &v){
  double avg_u = 0, avg_v = 0;
  for(std::vector<cv::Point2f>::iterator it=p.begin(); it<p.end(); it++){
    avg_u += (*it).x;
    avg_v += (*it).y;
  }

  cv::Point2f center;
  center.x = avg_u/4.;
  center.y = avg_v/4.;

  for(std::vector<cv::Point2f>::iterator it=p.begin(); it<p.end(); it++){
    double u_dif = (*it).x - center.x;
    double v_dif = (*it).y - center.y;

    if(u_dif < 0 && v_dif < 0){
      v[0] = (*it);
    }else if(u_dif > 0 && v_dif < 0){
      v[1] = (*it);
    }else if(u_dif > 0 && v_dif > 0){
      v[2] = (*it);
    }else{
      v[3] = (*it);
    }
  }
}

std::vector<double> eigenMatrix2SixDOF(Eigen::Matrix4d transform_matrix){
  tf::Matrix3x3 tf3d;
  tf3d.setValue(transform_matrix(0,0), transform_matrix(0,1), transform_matrix(0,2),
  transform_matrix(1,0), transform_matrix(1,1), transform_matrix(1,2),
  transform_matrix(2,0), transform_matrix(2,1), transform_matrix(2,2));

  // if(DEBUG) ROS_INFO("Final Transformation");
  // if(DEBUG) cout << transform_matrix << endl;

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  #ifdef TF2

  // static tf2_ros::TransformBroadcaster br;
  // geometry_msgs::TransformStamped transformStamped;

  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = "velodyne";
  // transformStamped.child_frame_id = "stereo";
  // transformStamped.transform.translation.x = transform_matrix(0,3);
  // transformStamped.transform.translation.y = transform_matrix(1,3);
  // transformStamped.transform.translation.z = transform_matrix(2,3);
  // transformStamped.transform.rotation.x = tfqt.x();
  // transformStamped.transform.rotation.y = tfqt.y();
  // transformStamped.transform.rotation.z = tfqt.z();
  // transformStamped.transform.rotation.w = tfqt.w();

  // br.sendTransform(transformStamped);

  #else

  tf::Vector3 origin;
  origin.setValue(transform_matrix(0,3),transform_matrix(1,3),transform_matrix(2,3));

  tf::Transform transf;
  transf.setOrigin(origin);
  transf.setRotation(tfqt);

  #endif

  // Transformation matrix from stereo to lidar frame
  static tf::TransformBroadcaster br;
  tf::StampedTransform tf_velodyne2camera;  

  tf_velodyne2camera = tf::StampedTransform(transf, ros::Time::now(), "stereo", "livox");
  // if (publish_tf_) br.sendTransform(tf_velodyne2camera);

  // The medium transformation matrix from lidar to stereo frame.
  // tf::Transform inverse = tf_velodyne2camera.inverse();
  double roll, pitch, yaw;
  double xt = tf_velodyne2camera.getOrigin().getX(), yt = tf_velodyne2camera.getOrigin().getY(), zt = tf_velodyne2camera.getOrigin().getZ();
  tf_velodyne2camera.getBasis().getRPY(roll, pitch, yaw);
  // if(roll < 0){ roll += M_PI; }
  // if(pitch < 0){  pitch += M_PI;}
  // if(yaw < 0){ yaw += M_PI;}
  

  std::vector<double> params_6dof = {xt, yt, zt, roll, pitch, yaw};
  return params_6dof;
}

void projectVelo2Cam(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_3d_in, cv::Mat cameraMatrix, cv::Mat Tr_velo_to_cam, std::vector<cv::Point2f> &projected_points){
  // cameraMatrix: 3x3 matrix, camera intrinsic param
  // Tr_velo_to_cam: 4x4 matrix, extrinsic param from lidar to camera

  // pcl::PointCloud<pcl::PointXYZ>::Ptr

  // cv::Mat R,  T, size, cameraMatrix;
  // // Calculate transformation from velo to cam
  // transpose( R_c_to_v, R_v_to_c );
  // T_v_to_c = -R_v_to_c * T_c_to_v;
  // Tr_cam_to_velo = (Mat_<double>(4,4) <<
  //     R_c_to_v.at<double>(0, 0), R_c_to_v.at<double>(0, 1), R_c_to_v.at<double>(0, 2), T_c_to_v.at<double>(0),
  //     R_c_to_v.at<double>(1, 0), R_c_to_v.at<double>(1, 1), R_c_to_v.at<double>(1, 2), T_c_to_v.at<double>(1),
  //     R_c_to_v.at<double>(2, 0), R_c_to_v.at<double>(2, 1), R_c_to_v.at<double>(2, 2), T_c_to_v.at<double>(2),
  //     0, 0, 0, 1
  // ); 
  // Tr_velo_to_cam = Tr_cam_to_velo.inv();

  projected_points.clear();
  // camera matrix K (3x4)
  cv::Mat K;
  K = (Mat_<double>(3,4) <<
          cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(0, 1), cameraMatrix.at<double>(0, 2), 0,
          cameraMatrix.at<double>(1, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(1, 2), 0,
          cameraMatrix.at<double>(2, 0), cameraMatrix.at<double>(2, 1), cameraMatrix.at<double>(2, 2), 0
          );

  // final projection matrix (3x4)
  cv::Mat P_velo_to_img;
  P_velo_to_img = K  * Tr_velo_to_cam; 

  for(auto it=point_3d_in->points.begin(); it<point_3d_in->points.end(); it++){

    // convert "objectPoints" to homogeneous "ptMat"
    // cv::Point3f pt = objectPoints[i];
    cv::Mat ptMat;
    ptMat = (Mat_<double>(4, 1) << (*it).x , (*it).y , (*it).z , 1);    

    // Perform matrix multiplication and save as Mat_ for easy element access
    cv::Mat dstMat;
    dstMat= P_velo_to_img * ptMat;      // multiple by projection matrix

    // Divide first 2 elements by the 3rd and assign to Point2f
    double scale = dstMat.at<double>(2, 0);
    if (scale < 10e-6)
        scale  = 10e-6;
    cv::Point2f dst(dstMat.at<double>(0, 0) / scale, dstMat.at<double>(1, 0) / scale);        

    // ------- added by Pop ----------// 
    // if ( dst.x >= 0 && dst.y >= 0 && dst.x < depthImage.cols && dst.y < depthImage.rows)  
    // {
    //     // cout << "dst[" << i << "]=" << dst.y << " " << dst.x << endl;
    //     depthImage.at<float>(dst.y, dst.x) = pt.x;  // should be pt.x, since in Velo, pt.x is depth infomation
    // }

    projected_points.push_back(dst);

  } 

}

const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &tstruct);

  return buf;
}

#endif