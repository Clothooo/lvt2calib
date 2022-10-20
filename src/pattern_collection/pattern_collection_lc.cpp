#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <ctime>
#include "tinyxml.h"
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lvt2calib/ClusterCentroids.h>
#include <lvt2calib/Cam2DCircleCenters.h>
#include <lvt2calib/EstimateBoundary.h>
#include <lvt2calib/lvt2_utlis.h>

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

#define DEBUG 0

using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace Eigen;

pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, camera_cloud;
std::vector<pcl::PointXYZ> lv(4), camv(4);

std::vector<cv::Point2f>  cam_2d_centers(4), cam_2d_centers_sorted(4);  // centers
std::vector<std::vector<cv::Point2f>> acc_cv_2d(4);
pcl::PointCloud<pcl::PointXYZ>::Ptr acc_laser_cloud, acc_camera_cloud;
std::vector<std::vector<pcl::PointXYZ>> acc_lv(4), acc_cv(4);  // accumulate 4 center points

std::vector<pcl::PointXYZ> final_centroid_acc_lv, final_centroid_acc_cv;
std::vector<cv::Point2f> final_cam_2d_centers_sorted(4);  // centers

ros::Publisher acc_laser_cloud_pub, acc_camera_pc_pub_;

bool useCentroid_laser = false;
bool save_final_data = false;

bool laserReceived, cameraReceived, cam2dReceived;
bool laser_end = false, cam_end = false, final_saved = false;

string result_dir_, feature_file_name_;
string ns_lv, ns_cv;
ostringstream os_final, os_final_realtime;
int max_frame = 10;
int acc_cam_frame = 0, acc_laser_frame = 0;

void recordFeature_laser(int acc_frame);
void recordFeature_cam();
void fileHandle();
void writeCircleCenters(const char* file_name, vector<pcl::PointXYZ>& centers_v_laser, vector<pcl::PointXYZ>& centers_v_cam_3d, vector<cv::Point2f>& centers_v_cam_2d);

void laser_callback(const lvt2calib::ClusterCentroids::ConstPtr livox_centroids)
{
    if(DEBUG) ROS_INFO("[%s] Laser pattern ready!", ns_lv.c_str());
    laserReceived = true;

    if(DEBUG) cout << "[" << ns_lv << "] livox_centroids->cloud.size = " << livox_centroids->cloud.width << endl;
    fromROSMsg(livox_centroids->cloud, *laser_cloud);

    sortPatternCentersYZ(laser_cloud, lv);  // sort by coordinates
    if(DEBUG) cout << "[" << ns_lv << "] laser_cloud.size = " << laser_cloud->points.size() << endl;

 
    if(DEBUG) 
    {
        ROS_INFO("[L2C] LASER");
        for(vector<pcl::PointXYZ>::iterator it=lv.begin(); it<lv.end(); ++it)
            cout << "l" << it - lv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
    }
    if(laserReceived)
        recordFeature_laser(livox_centroids -> cluster_iterations);

    sensor_msgs::PointCloud2 acc_laser_cloud_ros;
    pcl::toROSMsg(*acc_laser_cloud, acc_laser_cloud_ros);
    acc_laser_cloud_ros.header = livox_centroids->header;
    acc_laser_cloud_pub.publish(acc_laser_cloud_ros);  // Topic: /pattern_collection/acc_laser_cloud

    return;
}

void camera_callback(lvt2calib::ClusterCentroids::ConstPtr image_centroids)
{
    if(DEBUG) ROS_INFO("[%s] Camera pattern ready!", ns_cv.c_str());
    #ifdef TF2

    //TODO: adapt to ClusterCentroids

    PointCloud2 xy_image_cloud;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("stereo", "stereo_camera",
                            ros::Time(0), ros::Duration(20));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
        tf2::doTransform (*image_cloud, xy_image_cloud, transformStamped);
        fromROSMsg(xy_image_cloud, *camera_cloud);

    #else

    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_camera_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    fromROSMsg(image_centroids->cloud, *xy_camera_cloud);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        if(DEBUG) cout << "[camera_callback] listen for tf" << endl;
        listener.waitForTransform("stereo", "stereo_camera", ros::Time(0), ros::Duration(20.0));
        listener.lookupTransform ("stereo", "stereo_camera", ros::Time(0), transform);
    }catch (tf::TransformException& ex) {
        ROS_WARN("[%s] TF exception:\n%s", ns_cv.c_str(), ex.what());
        return;
    }
    pcl_ros::transformPointCloud (*xy_camera_cloud, *camera_cloud, transform);

    #endif

    cameraReceived = true;
    sortPatternCentersYZ(camera_cloud, camv);

    if(DEBUG) 
    {
        ROS_INFO("[L2C] CAMERA");
        for(vector<pcl::PointXYZ>::iterator it=camv.begin(); it<camv.end(); ++it)
            cout << "c" << it - camv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]"<<endl;
    }
    // if(cameraReceived && cam2dReceived && !cam_end)
    if(cameraReceived && cam2dReceived)
        recordFeature_cam();
}


void cam_2d_callback(const lvt2calib::Cam2DCircleCenters::ConstPtr cam_2d_circle_centers_msg){
  
  if(DEBUG) ROS_INFO("[%s] Camera 2d pattern ready!", ns_cv.c_str());
  if (cam_2d_circle_centers_msg->points.size() != 4){
    ROS_ERROR("[%s] Not exactly 4 2d circle centers from camera!", ns_cv.c_str());
    return;
  }

  if(DEBUG) cout << "[" << ns_cv << "] 2d centers size:" << cam_2d_circle_centers_msg->points.size() << endl;
  for(int i=0; i<cam_2d_circle_centers_msg->points.size(); i++){
    cam_2d_centers[i].x = cam_2d_circle_centers_msg->points[i].x;
    cam_2d_centers[i].y = cam_2d_circle_centers_msg->points[i].y;
  }

  sortPatternCentersUV(cam_2d_centers, cam_2d_centers_sorted);
  cam2dReceived = true;
}

void recordFeature_laser(int acc_frame)
{
    if(!laser_end)
    {
        acc_laser_frame = acc_frame;
        // ROS_WARN("***************************************");
        // ROS_WARN("[%s] Record Features......[FRAME: %d/%d]", ns_lv.c_str(), acc_frame, max_frame);
        ROS_WARN("[%s] Record Features......[%s: %d/%d %s: %d/%d]", ns_lv.c_str(), ns_lv.c_str(), acc_laser_frame, max_frame, ns_cv.c_str(), acc_cam_frame, max_frame);
        // ROS_WARN("***************************************");

        std::vector<pcl::PointXYZ> local_lv;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_laser_cloud;
        pcl::PointCloud<pcl::PointXYZ> local_l_cloud;

        local_lv = lv;
        local_laser_cloud = laser_cloud;
        *acc_laser_cloud += *local_laser_cloud;

        std::vector<pcl::PointXYZ> centroid_acc_lv;
        if (useCentroid_laser)
        {
            if(DEBUG) ROS_WARN("[%s] A. Use centroid!", ns_lv.c_str());
            // 4 center clusters
            acc_lv[0].push_back(local_lv[0]);
            acc_lv[1].push_back(local_lv[1]);
            acc_lv[2].push_back(local_lv[2]);
            acc_lv[3].push_back(local_lv[3]);

            if(DEBUG)
            {
                cout << "cluster acc_lv[0] size = " << acc_lv[0].size() << endl;
                cout << "cluster acc_lv[1] size = " << acc_lv[1].size() << endl;
                cout << "cluster acc_lv[2] size = " << acc_lv[2].size() << endl;
                cout << "cluster acc_lv[3] size = " << acc_lv[3].size() << endl;
            }

            if(DEBUG) cout << "**** [" << ns_lv << "] A.1. get four centroid points of camera" << endl;
            pcl::PointXYZ centroid_lv_0 = calculateClusterCentroid(acc_lv[0]);
            pcl::PointXYZ centroid_lv_1 = calculateClusterCentroid(acc_lv[1]);
            pcl::PointXYZ centroid_lv_2 = calculateClusterCentroid(acc_lv[2]);
            pcl::PointXYZ centroid_lv_3 = calculateClusterCentroid(acc_lv[3]);
            if(DEBUG)
            {
                cout << "centroid_lv_0 = " << centroid_lv_0.x << ", " << centroid_lv_0.y << ", " << centroid_lv_0.z << endl;
                cout << "centroid_lv_1 = " << centroid_lv_1.x << ", " << centroid_lv_1.y << ", " << centroid_lv_1.z << endl;
                cout << "centroid_lv_2 = " << centroid_lv_2.x << ", " << centroid_lv_2.y << ", " << centroid_lv_2.z << endl;
                cout << "centroid_lv_3 = " << centroid_lv_3.x << ", " << centroid_lv_3.y << ", " << centroid_lv_3.z << endl;
            }
            // [four centroids]
            centroid_acc_lv = {centroid_lv_0, centroid_lv_1, centroid_lv_2, centroid_lv_3};
        }
        else
        {
            if(DEBUG) ROS_WARN("[%s] B. Don't use centroid!", ns_lv.c_str());
            centroid_acc_lv = {local_lv[0], local_lv[1], local_lv[2], local_lv[3]};
        }

        if(DEBUG)
            for(vector<pcl::PointXYZ>::iterator it=centroid_acc_lv.begin(); it<centroid_acc_lv.end(); ++it){
                cout << "detected_3d_lidar_wt_centroid" << it - centroid_acc_lv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
            }

        std::vector<double> rmse_3d_lidar_wt_centroid;
    
        if(acc_frame == max_frame)
        {
            laser_end = true;
            final_centroid_acc_lv = centroid_acc_lv;

            ROS_WARN("[%s] REACH THE MAX FRMAE", ns_lv.c_str());
            if(save_final_data && laser_end && cam_end)
            {
                ROS_INFO("<<< Saving Data...");
                writeCircleCenters(os_final.str().c_str(), final_centroid_acc_lv, final_centroid_acc_cv, final_cam_2d_centers_sorted);
                writeCircleCenters(os_final_realtime.str().c_str(), final_centroid_acc_lv, final_centroid_acc_cv, final_cam_2d_centers_sorted);

                laser_end = false;
                cam_end = false;
                final_saved =true;
                ROS_WARN("****** Final Data Saved!!! (%s) ******", ns_lv.c_str());

                sleep(2);

            }
        }
    }
    laserReceived = false;
}

void recordFeature_cam()
{
    if(!cam_end)
    {
        acc_cam_frame ++;
        // ROS_WARN("***************************************");
        ROS_WARN("[%s] Record Features......[%s: %d/%d %s: %d/%d]", ns_cv.c_str(), ns_cv.c_str(), acc_cam_frame, max_frame, ns_lv.c_str(), acc_laser_frame, max_frame);
        // ROS_WARN("***************************************");

        std::vector<pcl::PointXYZ> local_cv;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_camera_cloud;
        pcl::PointCloud<pcl::PointXYZ> local_c_cloud;

        local_cv = camv;
        local_camera_cloud = camera_cloud;
        
        acc_cv[0].push_back(local_cv[0]);
        acc_cv[1].push_back(local_cv[1]);
        acc_cv[2].push_back(local_cv[2]);
        acc_cv[3].push_back(local_cv[3]);
        if(DEBUG)
        {
            cout << "cluster acc_cv[0] size = " << acc_cv[0].size() << endl;
            cout << "cluster acc_cv[1] size = " << acc_cv[1].size() << endl;
            cout << "cluster acc_cv[2] size = " << acc_cv[2].size() << endl;
            cout << "cluster acc_cv[3] size = " << acc_cv[3].size() << endl;
        }
        acc_cv_2d[0].push_back(cam_2d_centers_sorted[0]);
        acc_cv_2d[1].push_back(cam_2d_centers_sorted[1]);
        acc_cv_2d[2].push_back(cam_2d_centers_sorted[2]);
        acc_cv_2d[3].push_back(cam_2d_centers_sorted[3]);
        if(DEBUG)
        {
            cout << "cluster acc_cv_2d[0] size = " << acc_cv_2d[0].size() << endl;
            cout << "cluster acc_cv_2d[1] size = " << acc_cv_2d[1].size() << endl;
            cout << "cluster acc_cv_2d[2] size = " << acc_cv_2d[2].size() << endl;
            cout << "cluster acc_cv_2d[3] size = " << acc_cv_2d[3].size() << endl;
        }

        if(DEBUG) cout << "**** [" << ns_cv << "] A.1. get four centroid points of camera" << endl;
        pcl::PointXYZ centroid_cv_0 = calculateClusterCentroid(acc_cv[0]);
        pcl::PointXYZ centroid_cv_1 = calculateClusterCentroid(acc_cv[1]);
        pcl::PointXYZ centroid_cv_2 = calculateClusterCentroid(acc_cv[2]);
        pcl::PointXYZ centroid_cv_3 = calculateClusterCentroid(acc_cv[3]);
        if(DEBUG)
        {
            cout << "centroid_cv_0 = " << centroid_cv_0.x << ", " << centroid_cv_0.y << ", " << centroid_cv_0.z << endl;
            cout << "centroid_cv_1 = " << centroid_cv_1.x << ", " << centroid_cv_1.y << ", " << centroid_cv_1.z << endl;
            cout << "centroid_cv_2 = " << centroid_cv_2.x << ", " << centroid_cv_2.y << ", " << centroid_cv_2.z << endl;
            cout << "centroid_cv_3 = " << centroid_cv_3.x << ", " << centroid_cv_3.y << ", " << centroid_cv_3.z << endl;
        }

        cv::Point2f centroid_cv_2d_0 = calculateClusterCentroid2d(acc_cv_2d[0]);
        cv::Point2f centroid_cv_2d_1 = calculateClusterCentroid2d(acc_cv_2d[1]);
        cv::Point2f centroid_cv_2d_2 = calculateClusterCentroid2d(acc_cv_2d[2]);
        cv::Point2f centroid_cv_2d_3 = calculateClusterCentroid2d(acc_cv_2d[3]);
        if(DEBUG)
        {
            cout << "centroid_cv_2d_0 = " << centroid_cv_2d_0.x << ", " << centroid_cv_2d_0.y << endl;
            cout << "centroid_cv_2d_1 = " << centroid_cv_2d_1.x << ", " << centroid_cv_2d_1.y << endl;
            cout << "centroid_cv_2d_2 = " << centroid_cv_2d_2.x << ", " << centroid_cv_2d_2.y << endl;
            cout << "centroid_cv_2d_3 = " << centroid_cv_2d_3.x << ", " << centroid_cv_2d_3.y << endl;
        }
        // [four centroids]
        std::vector<pcl::PointXYZ> centroid_acc_cv = {centroid_cv_0, centroid_cv_1, centroid_cv_2, centroid_cv_3};
        std::vector<cv::Point2f> centroid_acc_cv_2d = {centroid_cv_2d_0, centroid_cv_2d_1, centroid_cv_2d_2, centroid_cv_2d_3};
        *acc_camera_cloud += *local_camera_cloud;

        if(DEBUG)
            for(vector<pcl::PointXYZ>::iterator it=centroid_acc_cv.begin(); it<centroid_acc_cv.end(); ++it){
                cout << "detected_3d_cam_wt_centroid" << it - centroid_acc_cv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
            }

        if(acc_cam_frame == max_frame)
        {
            cam_end = true;
            final_centroid_acc_cv = centroid_acc_cv;
            final_cam_2d_centers_sorted = centroid_acc_cv_2d;

            ROS_WARN("[%s] REACH THE MAX FRMAE", ns_cv.c_str());
            if(save_final_data && laser_end && cam_end)
            {
                ROS_INFO("<<< Saving Data...");
                writeCircleCenters(os_final.str().c_str(), final_centroid_acc_lv, final_centroid_acc_cv, final_cam_2d_centers_sorted);
                writeCircleCenters(os_final_realtime.str().c_str(), final_centroid_acc_lv, final_centroid_acc_cv, final_cam_2d_centers_sorted);

                laser_end = false;
                cam_end = false;
                final_saved =true;
                ROS_WARN("****** Final Data Saved!!! (%s) ******", ns_cv.c_str());
                sleep(2);
            }
        }
    }

    cameraReceived = false;
    cam2dReceived = false;
}


void fileHandle()
{
    if(save_final_data)
    {
        os_final.str("");
        os_final_realtime.str("");
        os_final << result_dir_ << feature_file_name_ << ".csv";
        os_final_realtime << result_dir_ << feature_file_name_ << "_" << currentDateTime() << ".csv" << endl;

        ROS_INFO("opening %s", os_final.str().c_str());
        ROS_INFO("opening %s", os_final_realtime.str().c_str());

        std::ofstream of_final_centers, of_final_centers_realtime;
        of_final_centers.open(os_final.str().c_str());
        of_final_centers_realtime.open(os_final_realtime.str().c_str());
        of_final_centers << "time,detected_lv[0]x,detected_lv[0]y,detected_lv[0]z,detected_lv[1]x,detected_lv[1]y,detected_lv[1]z,detected_lv[2]x,detected_lv[2]y,detected_lv[2]z,detected_lv[3]x,detected_lv[3]y,detected_lv[3]z,detected_cv[0]x,detected_cv[0]y,detected_cv[0]z,detected_cv[1]x,detected_cv[1]y,detected_cv[1]z,detected_cv[2]x,detected_cv[2]y,detected_cv[2]z,detected_cv[3]x,detected_cv[3]y,detected_cv[3]z,cam_2d_detected_centers[0]x,cam_2d_detected_centers[0]y,cam_2d_detected_centers[1]x,cam_2d_detected_centers[1]y,cam_2d_detected_centers[2]x,cam_2d_detected_centers[2]y,cam_2d_detected_centers[3]x,cam_2d_detected_centers[3]y" << endl;
        of_final_centers.close();
        of_final_centers_realtime << "time,detected_lv[0]x,detected_lv[0]y,detected_lv[0]z,detected_lv[1]x,detected_lv[1]y,detected_lv[1]z,detected_lv[2]x,detected_lv[2]y,detected_lv[2]z,detected_lv[3]x,detected_lv[3]y,detected_lv[3]z,detected_cv[0]x,detected_cv[0]y,detected_cv[0]z,detected_cv[1]x,detected_cv[1]y,detected_cv[1]z,detected_cv[2]x,detected_cv[2]y,detected_cv[2]z,detected_cv[3]x,detected_cv[3]y,detected_cv[3]z,cam_2d_detected_centers[0]x,cam_2d_detected_centers[0]y,cam_2d_detected_centers[1]x,cam_2d_detected_centers[1]y,cam_2d_detected_centers[2]x,cam_2d_detected_centers[2]y,cam_2d_detected_centers[3]x,cam_2d_detected_centers[3]y" << endl;
        of_final_centers_realtime.close();
    }
}

void writeCircleCenters(const char* file_name, vector<pcl::PointXYZ>& centers_v_laser, vector<pcl::PointXYZ>& centers_v_cam_3d, vector<cv::Point2f>& centers_v_cam_2d)
{
    std::ofstream of_file;
    of_file.open(file_name, ios::app);

    of_file << currentDateTime();

    for(int i = 0; i < 4; i++)
        of_file << "," << centers_v_laser[i].x << "," << centers_v_laser[i].y << "," << centers_v_laser[i].z;
    
    for(int i = 0; i < 4; i++)
        of_file << "," << centers_v_cam_3d[i].x << "," << centers_v_cam_3d[i].y << "," << centers_v_cam_3d[i].z;
    
    for(int i = 0; i < 4; i++)
        of_file << "," << centers_v_cam_2d[i].x << "," << centers_v_cam_2d[i].y;

    of_file << endl;
    of_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pattern_collection_lc");
    ros::NodeHandle nh_("~");

    nh_.param<bool>("useCentroid_laser", useCentroid_laser, false);
    nh_.param<bool>("save_final_data", save_final_data, false);
    
    nh_.param<string>("result_dir_", result_dir_, "");
    nh_.param<string>("feature_file_name",feature_file_name_, "");
    nh_.param<string>("ns_lv", ns_lv, "LASER");
    nh_.param<string>("ns_cv", ns_cv, "CAMERA");
    ros::param::get("/max_frame", max_frame);
    
    laserReceived = false;
    laser_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cameraReceived = false;
    camera_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    acc_laser_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    acc_camera_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Subscriber laser_sub = nh_.subscribe<lvt2calib::ClusterCentroids>("cloud_laser", 1, laser_callback);
    ros::Subscriber stereo_sub = nh_.subscribe<lvt2calib::ClusterCentroids>("cloud_cam", 1, camera_callback);
    ros::Subscriber stereo_2d_circle_centers_sub = nh_.subscribe<lvt2calib::Cam2DCircleCenters>("cloud_cam2d", 1, cam_2d_callback);

    acc_laser_cloud_pub = nh_.advertise<PointCloud2>("acc_laser_centers",1);

    ROS_INFO("Initialized!");
    int posNo = 0;
    ros::Rate loop_rate(30);
   
    pcl::console::TicToc t_process;
    double Process_time_ = 0.0;

    ROS_WARN("<<<<<<<<<<<< [COLLECT] READY? <<<<<<<<<<<<");
    ROS_INFO("----- If yes, please press 'Y' and 'ENTER'!");
    ROS_INFO("----- If want to quit, please press 'N' and 'ENTER'!");
    char key;
    cin >> key;
    if(key == 'Y' || key == 'y')
    {
        ROS_INFO("----- Continue..."); 
        fileHandle();
        t_process.tic();
        
        while(ros::ok())
        {
            if(!useCentroid_laser)
                ros::param::set("/do_acc_boards", true);
            else
                ros::param::set("/do_acc_boards", false);
            ros::spinOnce();

            if(final_saved)
            {
                Process_time_ = t_process.toc();

                ros::param::set("/pause_process", true);
                ros::param::set("/do_acc_boards", false);
    
                ROS_WARN("<<<<<<<<<<<< [COLLECT] PROCESS FINISHED! <<<<<<<<<<<<");
                ROS_WARN("<<<<<<<<<<<< COST TIME: %fs", (float) Process_time_ / 1000);
                ROS_INFO("Have processed %d positions. Need to collect patterns in the next position?", posNo + 1);
                ROS_INFO("-----If yes, please change the position or change the rosbag, then press 'Y' and 'ENTER'!");
                ROS_INFO("-----If no, please press 'N' and 'ENTER' to quit the process!");
                char key;
                cin >> key;
                if(key == 'Y' || key == 'y')
                {
                    ros::param::set("/pause_process", false);
                    ROS_WARN("<<<<<<<<<<<< CHANGE POSITION <<<<<<<<<<<<");
                    ROS_WARN("<<<<<<<<<<<< [COLLECT] READY? <<<<<<<<<<<<");
                    ROS_INFO("----- If yes, please press 'Y' and 'ENTER'!");
                    ROS_INFO("----- If want to quit, please press 'N' and 'ENTER'!");
                    char key;
                    cin >> key;
                    if(key == 'Y' || key == 'y')
                    {
                        ROS_INFO("----- Continue...");
                        t_process.tic();
                    }
                    else
                    {
                        ROS_WARN("<<<<<<<<<<<< END <<<<<<<<<<<<");
                        ros::param::set("/end_process", true);
                        break;
                    }

                    // ros::param::set("bag_changed", true);
                    posNo ++;
                    
                    final_saved = false;
                    acc_cam_frame = 0;
                    acc_cv_2d[0].clear();
                    acc_cv_2d[1].clear();
                    acc_cv_2d[2].clear();
                    acc_cv_2d[3].clear();
                    acc_cv[0].clear();
                    acc_cv[1].clear();
                    acc_cv[2].clear();
                    acc_cv[3].clear();
                    acc_lv[0].clear();
                    acc_lv[1].clear();
                    acc_lv[2].clear();
                    acc_lv[3].clear();
                    acc_laser_cloud->clear();
                    acc_camera_cloud->clear();  

                    ros::param::set("/do_acc_boards", true);
                }  
                // else if(key == 'n' || key == 'N')
                else
                {
                    ROS_WARN("<<<<<<<<<<<< END <<<<<<<<<<<<");
                    ros::param::set("/end_process", true);
                    break;
                }
            }   
        }
    }
    else
    {
        ROS_WARN("<<<<<<<<<<<< END <<<<<<<<<<<<");
        ros::param::set("/end_process", true);
    }
    ROS_WARN("Features saved in:\n%s\n%s", os_final.str().c_str(), os_final_realtime.str().c_str());

    ros::shutdown();
    return 0;
}