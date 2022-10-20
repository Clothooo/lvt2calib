#ifndef lvt2Calib_H
#define lvt2Calib_H

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
#include <list>
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

#include "excalib_min2d.h"
#include "lvt2_utlis.h"

using namespace std;
using namespace cv;
using namespace pcl;

// void sortPatternCentersYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, vector<pcl::PointXYZ> &v);
// pcl::PointXYZ calculateClusterCentroid(std::vector<pcl::PointXYZ> one_acc_points);
// cv::Point2f calculateClusterCentroid2d(std::vector<cv::Point2f> one_acc_points);
// std::vector<double> calculateRMSE(std::vector<pcl::PointXYZ> ground_truth, std::vector<pcl::PointXYZ> detected);

enum calibType {L2C_CALIB = 1, L2L_CALIB};

struct poi
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor1_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor2_points; // l2c: camera_points
    std::vector<cv::Point2f> camera_2d;
    poi()
    {
        sensor1_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        sensor2_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        camera_2d.resize(4);
    }
    poi(std::vector<cv::Point2f> camera_2d_, pcl::PointCloud<pcl::PointXYZ> laser_points_, pcl::PointCloud<pcl::PointXYZ> camera_points_)
    {
        sensor1_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(laser_points_));
        sensor2_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(camera_points_));
        camera_2d = camera_2d_;
    }
    poi(pcl::PointCloud<pcl::PointXYZ> laser1_points_, pcl::PointCloud<pcl::PointXYZ> laser2_points_)
    {
        sensor1_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(laser1_points_));
        sensor2_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(laser2_points_));
    }
};

class lvt2Calib 
{
    public:
    // std::vector<poi> samples;
    std::vector<poi> feature_points;
    Eigen::Matrix3d cameraMatrix_;
    tf::Transform transf_2d;
    tf::Transform transf_3d;
    tf::Transform transf_gt;
    tf::StampedTransform tf_livox_cam_min2d;
    tf::StampedTransform tf_livox_cam_min3d;
    tf::StampedTransform tf_livox_cam_gt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr s1_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr s2_cloud; // camera points
    std::vector<cv::Point2f> cam_2d_points;

    lvt2Calib(){
      s1_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      s2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    };
    lvt2Calib(calibType calib_type_){
      switch (calib_type_)
      {
      case L2C_CALIB:
          isl2lCalib_ = false;
          break;
      case L2L_CALIB:
          isl2lCalib_ = true;
          break;        
      default:
          isl2lCalib_ = false;
          break;
      }

      s1_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      s2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    }
    lvt2Calib(Eigen::Matrix3d cameraMatrix)
    {
        cameraMatrix_ = cameraMatrix;
    }
    void toPublishTF_GT(bool flag){
        publish_tf_gt_ = flag;
    }
    void toUseCalibMin3D(bool flag){
        useCalibMin3D_ = flag;
    }
    void toUseCalibMin2D(bool flag){
        useCalibMin2D_ = flag;
    }
    int featureSize(){
        return feature_points.size();
    }

    
    bool loadCSV(const char* filename);
    Eigen::Matrix4d ExtCalib3D(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud);
    Eigen::Matrix4d ExtCalib2D(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, std::vector<cv::Point2f> cam_2d_sorted, Eigen::Matrix4d init_ext);
    std::vector<double> calAlignError(pcl::PointCloud<pcl::PointXYZ>::Ptr laser1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr laser2_cloud, Eigen::Matrix4d Tr_l1tol2);
    Eigen::Matrix4f obtainGT_GAZEBO();
    void randSample(int sample_size);

    private:
    bool publish_tf_gt_ = false;
    bool useCalibMin3D_ = false;
    bool useCalibMin2D_ = false;
    bool isl2lCalib_ = false;

};

bool lvt2Calib::loadCSV(const char* filename)
{
    ifstream loadfile;
    loadfile.open(filename);
    ROS_INFO("<<<<<<<<<<< LOADING FILE %s", filename);
    if(!loadfile)
    {
        ROS_WARN("Opening file faild!");
        return false;
    }
    int i = 0;
    std::string line;
    pcl::PointXYZ centroid_s1, centroid_s2; // sensor1, sensor2
    cv::Point2f cam_2d_center;

    while (getline(loadfile, line))
    {
        // cout << line << endl;
        if(DEBUG) cout << "line " << i << endl; 
        if(i==0) {
            i++;
            continue;
        }

        std::istringstream sin(line);
        std::vector<string> fileds;
        std::string filed;
        while(getline(sin,filed,','))
        {
            fileds.push_back(filed);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr fourcircle_s1(new pcl::PointCloud<pcl::PointXYZ>),
                                            fourcircle_s2(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<cv::Point2f> cam_2d;

        for(int j = 0; j < 4; j++)
        {
            centroid_s1.x = atof(fileds[3*j+1].c_str());
            centroid_s1.y = atof(fileds[3*j+2].c_str());
            centroid_s1.z = atof(fileds[3*j+3].c_str());
            centroid_s2.x = atof(fileds[3*j+12+1].c_str());
            centroid_s2.y = atof(fileds[3*j+12+2].c_str());
            centroid_s2.z = atof(fileds[3*j+12+3].c_str());
            if(!isl2lCalib_) // l2cCalib
            {
                cam_2d_center.x = atof(fileds[2*j+24+1].c_str());
                cam_2d_center.y = atof(fileds[2*j+24+2].c_str());
                cam_2d.push_back(cam_2d_center);
                cam_2d_points.push_back(cam_2d_center);
            }
            s1_cloud->points.push_back(centroid_s1);
            s2_cloud->points.push_back(centroid_s2);

            fourcircle_s1->points.push_back(centroid_s1);
            fourcircle_s2->points.push_back(centroid_s2);
            
        }
        poi four_centers(*fourcircle_s1, *fourcircle_s2);
        if(!isl2lCalib_) four_centers.camera_2d = cam_2d;

        if(DEBUG)
        {
            cout << "POS " << i << ":" << endl;
            cout << "lv_points:" << endl;
            for(auto p : four_centers.sensor1_points->points)
            {
                cout << "[ " << p.x << ", " << p.y << ", " << p.z << " ]" << endl;
            }
            cout << "cv_points:" << endl;
            for(auto p : four_centers.sensor2_points->points)
            {
                cout << "[ " << p.x << ", " << p.y << ", " << p.z << " ]" << endl;
            }
            cout << "cv_points_2d:" << endl;
            for(auto p:four_centers.camera_2d)
            {
                cout << "[ " << p.x << ", " << p.y << " ]" << endl;
            } 
        }
        
        feature_points.push_back(four_centers);

        i++;
    }
    if(DEBUG)
    {
        cout << "s1_cloud.size = " << s1_cloud->points.size() << endl;
        cout << "s2_cloud.size = " << s2_cloud->points.size() << endl;
        cout << "cam_2d_points.size = " << cam_2d_points.size() << endl;
    }
    // cout << "line num = " << i << endl;
    ROS_INFO("<<<<<<<<<<<<<<<<<< pos num: %d", feature_points.size());

    return true;
}

Eigen::Matrix4d lvt2Calib::ExtCalib3D(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud)
{
    int size_p = laser_cloud->points.size();
    VectorXd p_laser(size_p * 3);
    VectorXd p_cam(size_p * 3);
    for(int i = 0; i < size_p; i ++)
    {
        p_laser[3*i] = laser_cloud->points[i].x;
        p_laser[3*i+1] = laser_cloud->points[i].y;
        p_laser[3*i+2] = laser_cloud->points[i].z;

        p_cam[3*i] = camera_cloud->points[i].x;
        p_cam[3*i+1] = camera_cloud->points[i].y;
        p_cam[3*i+2] = camera_cloud->points[i].z;
    }
    if(DEBUG)
    {
        cout << "p_laser.size = " << p_laser.size() << endl;
        cout << "p_cam.size = " << p_cam.size() << endl;
    }


    VectorXd diff = p_laser - p_cam;
    // cout << "diff.size = " << diff.size() << endl;

    Eigen::MatrixXd matrix_transl(3,3);
    for(int i = 1; i <= size_p; i++)
    {   
        if(i == 1)
        {
            matrix_transl = MatrixXd::Identity(3,3);
        }
        else
        {
            Eigen::MatrixXd tmp = matrix_transl;
            matrix_transl.resize(i*3,3);
            matrix_transl << tmp, 
                             MatrixXd::Identity(3,3);
        }

    }
    // cout << "matrix_transl.size = " << "\n" << matrix_transl.size() << endl;

    Eigen::Vector3d x;
    x = matrix_transl.colPivHouseholderQr().solve(diff); //  求解matrix_transl*x = diff_12中的x

    Eigen::Matrix4d Tm;
    Tm <<   1, 0, 0, x[0],
    0, 1, 0, x[1],
    0, 0, 1, x[2],
    0, 0, 0, 1;
    
    if(DEBUG) ROS_INFO("Step 1: Translation");
    if(DEBUG) cout << Tm << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr translated_pc (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*camera_cloud, *translated_pc, Tm);

    //  累积圆心点云ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(translated_pc);
    icp.setInputTarget(laser_cloud);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(1000);
    if (icp.hasConverged()){
        if(DEBUG) ROS_INFO("ICP Converged. Score: %lf", icp.getFitnessScore());
    }else{
        ROS_WARN("ICP failed to converge");
        // return 0;
        // break;
    }
    if(DEBUG) ROS_INFO("Step 2. ICP Transformation:");
    if(DEBUG) cout << icp.getFinalTransformation() << std::endl;

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    Eigen::Matrix4d final_trans = transformation * Tm;
    // Eigen::Matrix4d final_trans = transformation;

    tf::Matrix3x3 tf3d;
    tf3d.setValue(final_trans(0,0), final_trans(0,1), final_trans(0,2),
    final_trans(1,0), final_trans(1,1), final_trans(1,2),
    final_trans(2,0), final_trans(2,1), final_trans(2,2));

    if(DEBUG) ROS_INFO("Final Transformation");
    if(DEBUG) cout << final_trans << endl;

    // tf::Quaternion tfqt;
    // tf3d.getRotation(tfqt);

    return final_trans;
}

Eigen::Matrix4d lvt2Calib::ExtCalib2D(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, std::vector<cv::Point2f> cam_2d_sorted, Eigen::Matrix4d init_ext)
{
    Eigen::Matrix3d R;
    R << init_ext(0,0), init_ext(0,1), init_ext(0,2),
        init_ext(1,0), init_ext(1,1), init_ext(1,2),
        init_ext(2,0), init_ext(2,1), init_ext(2,2);
    Eigen::Quaterniond q(R);

    double ext[7];
    ext[0] = q.x();
    ext[1] = q.y();
    ext[2] = q.z();
    ext[3] = q.w();
    ext[4] = init_ext(0,3);  
    ext[5] = init_ext(1,3);  
    ext[6] = init_ext(2,3); 

    Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
    Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);
    
    if(DEBUG)
    {
        cout << "init rot = " << m_q.toRotationMatrix() << endl;
        cout << "init m_t = " << m_t << endl;
    }

    ceres::LocalParameterization * q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;

    problem.AddParameterBlock(ext, 4, q_parameterization);
    problem.AddParameterBlock(ext + 4, 3);

    int pointsize = laser_cloud->points.size();
    if(DEBUG) cout << "pointsize = " << pointsize << endl;
    for(int val=0 ; val < pointsize; val ++)
    {
        ceres::CostFunction *cost_function;
        cost_function = excalib_min2d::Create(laser_cloud->points[val], cam_2d_sorted[val], cameraMatrix_);
        problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if(DEBUG) cout << summary.BriefReport() << endl;

    Eigen::Matrix3d rot = m_q.toRotationMatrix();

    if(DEBUG)
    {
        cout << "rot = " << "\n" << rot << endl;
        cout << "m_t = " << "\n" << m_t << endl;
    }
    
    Eigen::Matrix4d temp_tr_v2c_2d;
    temp_tr_v2c_2d << rot(0,0), rot(0,1), rot(0,2), m_t[0],
                    rot(1,0), rot(1,1), rot(1,2), m_t[1],
                    rot(2,0), rot(2,1), rot(2,2), m_t[2],
                    0, 0, 0, 1;
    return temp_tr_v2c_2d;
}

std::vector<double> lvt2Calib::calAlignError(pcl::PointCloud<pcl::PointXYZ>::Ptr laser1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr laser2_cloud, Eigen::Matrix4d Tr_l1tol2)
{
    double sum = 0, sum_x = 0, sum_y = 0, sum_z = 0;
    double rmse_total = 0, rmse_x = 0, rmse_y = 0, rmse_z = 0;
    int point_size = laser1_cloud->points.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr l1_pc_in_l2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laser1_cloud, *l1_pc_in_l2, Tr_l1tol2);
    // cout << "[calAlignError] l1_pc_in_l2->points.size = " << l1_pc_in_l2->points.size() << endl;

    for(auto iter =laser2_cloud->points.begin(); iter <laser2_cloud->points.end(); iter++)
    {
        int idx = iter -laser2_cloud->points.begin();
        sum_x += pow(iter->x - l1_pc_in_l2->points[idx].x, 2);
        sum_y += pow(iter->y - l1_pc_in_l2->points[idx].y, 2);
        sum_z += pow(iter->z - l1_pc_in_l2->points[idx].z, 2);
    }

    rmse_x = sqrt(sum_x/double(point_size));
    rmse_y = sqrt(sum_y/double(point_size));
    rmse_z = sqrt(sum_z/double(point_size));
    rmse_total = sqrt(pow(rmse_x, 2) + pow(rmse_y, 2) + pow(rmse_z, 2));
    
    std::vector<double> rmse = {rmse_x, rmse_y, rmse_z, rmse_total};
    return rmse;
}

Eigen::Matrix4f lvt2Calib::obtainGT_GAZEBO()
{
    // ground truth 6dof params
    double roll_gt, pitch_gt, yaw_gt;
    double tx_gt, ty_gt, tz_gt;
    Eigen::Matrix4f Tr_v2s_gt, Tr_s2c_gt, Tr_v2c_gt;

    tf::TransformListener listener;
    tf::StampedTransform transform;  
    try{ 
        
        // stereo_camera_gt
        listener.waitForTransform("/stereo_gt", "/livox_gt", ros::Time(0), ros::Duration(5.0), ros::Duration(2));
        listener.lookupTransform("/stereo_gt", "/livox_gt",  
                                ros::Time(0), transform);
        // listener.lookupTransform("/stereo_gt", "/livox_gt",  
        //                         ros::Time::now(), transform);
                
        tx_gt = transform.getOrigin().getX();
        ty_gt = transform.getOrigin().getY();
        tz_gt = transform.getOrigin().getZ();

        if(DEBUG)
        {
            cout << " [V2S] Get /tf from /livox_gt to /stereo_gt: " << endl;
            cout << "translation =" << tx_gt << ", " << ty_gt << ", " << tz_gt << endl; 
        }
       

        transform.getBasis().getRPY(roll_gt, pitch_gt, yaw_gt);
        if(DEBUG)   cout << "euler angle =" << roll_gt << ", " << pitch_gt << ", " << yaw_gt << endl; 

        tf::Transform trans_v2s_gt = transform;
        Tr_v2s_gt << trans_v2s_gt.getBasis()[0][0], trans_v2s_gt.getBasis()[0][1], trans_v2s_gt.getBasis()[0][2], trans_v2s_gt.getOrigin().getX(),
        trans_v2s_gt.getBasis()[1][0], trans_v2s_gt.getBasis()[1][1], trans_v2s_gt.getBasis()[1][2], trans_v2s_gt.getOrigin().getY(),
        trans_v2s_gt.getBasis()[2][0], trans_v2s_gt.getBasis()[2][1], trans_v2s_gt.getBasis()[2][2], trans_v2s_gt.getOrigin().getZ(),
        0, 0, 0, 1;

        Tr_s2c_gt << 0, -1, 0, 0,
        0, 0, -1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;

        Tr_v2c_gt = Tr_s2c_gt * Tr_v2s_gt;

        if(DEBUG)   cout << "Tr_v2c_gt = " << "\n" << Tr_v2c_gt << endl;
        tf::Matrix3x3 tf3d;
        tf3d.setValue(Tr_v2c_gt(0,0), Tr_v2c_gt(0,1), Tr_v2c_gt(0,2),
        Tr_v2c_gt(1,0), Tr_v2c_gt(1,1), Tr_v2c_gt(1,2),
        Tr_v2c_gt(2,0), Tr_v2c_gt(2,1), Tr_v2c_gt(2,2));
        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        tf::Vector3 origin;
        origin.setValue(Tr_v2c_gt(0,3),Tr_v2c_gt(1,3),Tr_v2c_gt(2,3));

        transf_gt.setOrigin(origin);
        transf_gt.setRotation(tfqt);

        static tf::TransformBroadcaster br;
        tf_livox_cam_gt = tf::StampedTransform(transf_gt, ros::Time::now(), "livox", "camera");
        if (publish_tf_gt_) br.sendTransform(tf_livox_cam_gt);

    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return Tr_v2c_gt;
}


void RandSample(int begin, int end, int sample_size, int sample_num, list<list<int>>& ll)
{
    srand((unsigned)time(NULL));
    while (ll.size() < sample_num)
    {
        list<int> l;
        while(l.size()<sample_size)
        {
            l.push_back(rand() % (end - begin + 1) + begin);
            l.sort();
            l.unique();
        }
        ll.push_back(l);
        ll.sort();
        ll.unique();
    }
    return;
}

#endif