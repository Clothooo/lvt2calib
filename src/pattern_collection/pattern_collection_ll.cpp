#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

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
using namespace Eigen;

pcl::PointCloud<pcl::PointXYZ>::Ptr laser1_cloud, laser2_cloud;
std::vector<pcl::PointXYZ> lv1(4), lv2(4);

pcl::PointCloud<pcl::PointXYZ>::Ptr acc_laser1_cloud, acc_laser2_cloud;
std::vector<std::vector<pcl::PointXYZ>> acc_lv1(4), acc_lv2(4);  // accumulate 4 center points

std::vector<pcl::PointXYZ> final_centroid_acc_lv1;
std::vector<pcl::PointXYZ> final_centroid_acc_lv2;

ros::Publisher acc_laser1_cloud_pub, acc_laser2_cloud_pub;

bool useCentroid_laser = false;
bool save_final_data = false;

bool laser1Received, laser2Received;
bool laser1_end = false, laser2_end = false, final_saved = false;

string result_dir_ = "", feature_file_name_ = "";
string ns_l1, ns_l2;
ostringstream os_final, os_final_realtime;
int max_frame = 10;
int acc_laser1_frame = 0, acc_laser2_frame = 0;


void recordFeature_laser1(int acc_frame);
void recordFeature_laser2(int acc_frame);
void fileHandle();
void writeCircleCenters(const char* file_name, vector<pcl::PointXYZ>& centers_v1, vector<pcl::PointXYZ>& centers_v2);


void laser1_callback(const lvt2calib::ClusterCentroids::ConstPtr livox_centroids)
{
    if(DEBUG) ROS_INFO("[%s] pattern ready!", ns_l1.c_str());
    laser1Received = true;

    if(DEBUG) cout << "[" << ns_l1 << "] livox_centroids->cloud.size = " << livox_centroids->cloud.width << endl;
    fromROSMsg(livox_centroids->cloud, *laser1_cloud);

    sortPatternCentersYZ(laser1_cloud, lv1);  // sort by coordinates
    if(DEBUG) cout << "[" << ns_l1 << "] laser1_cloud.size = " << laser1_cloud->points.size() << endl;
 
    if(DEBUG) ROS_INFO("[L2L] %s", ns_l1.c_str());
    if(DEBUG)
        for(vector<pcl::PointXYZ>::iterator it=lv1.begin(); it<lv1.end(); ++it)
        {
            cout << "l" << it - lv1.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
        }

    if(laser1Received)
    {
        recordFeature_laser1(livox_centroids -> cluster_iterations);
    }

    sensor_msgs::PointCloud2 acc_laser_cloud_ros;
    pcl::toROSMsg(*acc_laser1_cloud, acc_laser_cloud_ros);
    acc_laser_cloud_ros.header = livox_centroids->header;
    acc_laser1_cloud_pub.publish(acc_laser_cloud_ros);  // Topic: /pattern_collection/acc_laser1_cloud

    return;
}

void laser2_callback(const lvt2calib::ClusterCentroids::ConstPtr livox_centroids)
{
    if(DEBUG) ROS_INFO("[%s] pattern ready!", ns_l2.c_str());
    laser2Received = true;

    if(DEBUG) cout << "[" << ns_l2 << "] livox_centroids->cloud.size = " << livox_centroids->cloud.width << endl;
    fromROSMsg(livox_centroids->cloud, *laser2_cloud);

    sortPatternCentersYZ(laser2_cloud, lv2);  //  利用相对于四个点的中心的yz坐标(世界坐标系）关系，给四个圆心点排序 
    if(DEBUG) cout << "[" << ns_l2 << "] laser2_cloud.size = " << laser2_cloud->points.size() << endl;
 
    if(DEBUG) ROS_INFO("[L2L] %s", ns_l2.c_str());
    if(DEBUG)
        for(vector<pcl::PointXYZ>::iterator it=lv2.begin(); it<lv2.end(); ++it)
        {
            cout << "l" << it - lv2.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
        }

    if(laser2Received)
    {
        recordFeature_laser2(livox_centroids -> cluster_iterations);
    }

    sensor_msgs::PointCloud2 acc_laser_cloud_ros;
    pcl::toROSMsg(*acc_laser2_cloud, acc_laser_cloud_ros);
    acc_laser_cloud_ros.header = livox_centroids->header;
    acc_laser2_cloud_pub.publish(acc_laser_cloud_ros);  // Topic: /pattern_collection/acc_laser1_cloud

    return;
}


void recordFeature_laser1(int acc_frame)
{
    if(!laser1_end)
    {
        acc_laser1_frame = acc_frame;
        // ROS_WARN("***************************************");
        ROS_WARN("[%s] Record Features......[%s: %d/%d %s: %d/%d]", ns_l1.c_str(), ns_l1.c_str(), acc_laser1_frame, max_frame, ns_l2.c_str(), acc_laser2_frame, max_frame);
        // ROS_WARN("***************************************");

        std::vector<pcl::PointXYZ> local_lv;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_laser_cloud;
        pcl::PointCloud<pcl::PointXYZ> local_l_cloud;

        local_lv = lv1;
        local_laser_cloud = laser1_cloud;
        *acc_laser1_cloud += *local_laser_cloud;

        std::vector<pcl::PointXYZ> centroid_acc_lv;
        if (useCentroid_laser)
        {
            if(DEBUG) ROS_WARN("[%s] A. Use centroid!", ns_l1.c_str());
            // 4 center clusters
            acc_lv1[0].push_back(local_lv[0]);
            acc_lv1[1].push_back(local_lv[1]);
            acc_lv1[2].push_back(local_lv[2]);
            acc_lv1[3].push_back(local_lv[3]);

            if(DEBUG)
            {
                cout << "cluster acc_lv1[0] size = " << acc_lv1[0].size() << endl;
                cout << "cluster acc_lv1[1] size = " << acc_lv1[1].size() << endl;
                cout << "cluster acc_lv1[2] size = " << acc_lv1[2].size() << endl;
                cout << "cluster acc_lv1[3] size = " << acc_lv1[3].size() << endl;
            }
            
            if(DEBUG) cout << "**** [" << ns_l1 << "] A.1. get four centroid points of laser" << endl;
            pcl::PointXYZ centroid_lv_0 = calculateClusterCentroid(acc_lv1[0]);
            pcl::PointXYZ centroid_lv_1 = calculateClusterCentroid(acc_lv1[1]);
            pcl::PointXYZ centroid_lv_2 = calculateClusterCentroid(acc_lv1[2]);
            pcl::PointXYZ centroid_lv_3 = calculateClusterCentroid(acc_lv1[3]);
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
            if(DEBUG) ROS_WARN("[%s] B. Don't use centroid!", ns_l1.c_str());
            centroid_acc_lv = {local_lv[0], local_lv[1], local_lv[2], local_lv[3]};
        }

        if(DEBUG)
            for(vector<pcl::PointXYZ>::iterator it=centroid_acc_lv.begin(); it<centroid_acc_lv.end(); ++it){
                cout << "detected_3d_lidar_wt_centroid" << it - centroid_acc_lv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
            }

        std::vector<double> rmse_3d_lidar_wt_centroid;
    
        if(acc_frame == max_frame)
        {
            laser1_end = true;
            final_centroid_acc_lv1 = centroid_acc_lv;

            ROS_WARN("[%s] REACH THE MAX FRMAE", ns_l1.c_str());
            if(save_final_data && laser1_end && laser2_end)
            {
                ROS_INFO("<<< Saving Data...");
                writeCircleCenters(os_final.str().c_str(), final_centroid_acc_lv1, final_centroid_acc_lv2);           
                writeCircleCenters(os_final_realtime.str().c_str(), final_centroid_acc_lv1, final_centroid_acc_lv2);           

                laser1_end = false;
                laser2_end = false;
                ROS_WARN("****** Final Data Saved!!! (%s) ******", ns_l1.c_str());
                final_saved =true;
                sleep(2);
            }
        }
    }
    laser1Received = false;
}

void recordFeature_laser2(int acc_frame)
{
    if(!laser2_end)
    {
        acc_laser2_frame = acc_frame;
        // ROS_WARN("***************************************");
        // ROS_WARN("[%s] Record Features........[FRAME: %d/%d]", ns_l2.c_str(), acc_frame, max_frame);
        ROS_WARN("[%s] Record Features......[%s: %d/%d %s: %d/%d]", ns_l2.c_str(), ns_l2.c_str(), acc_laser2_frame, max_frame, ns_l1.c_str(), acc_laser1_frame, max_frame);
        // ROS_WARN("***************************************");

        std::vector<pcl::PointXYZ> local_lv;
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_laser_cloud;
        pcl::PointCloud<pcl::PointXYZ> local_l_cloud;

        local_lv = lv2;
        local_laser_cloud = laser2_cloud;
        *acc_laser2_cloud += *local_laser_cloud;

        std::vector<pcl::PointXYZ> centroid_acc_lv;
        if (useCentroid_laser)
        {
            ROS_WARN("[%s] A. Use centroid!", ns_l2.c_str());
            // 4 center clusters
            acc_lv2[0].push_back(local_lv[0]);
            acc_lv2[1].push_back(local_lv[1]);
            acc_lv2[2].push_back(local_lv[2]);
            acc_lv2[3].push_back(local_lv[3]);

            cout << "cluster acc_lv2[0] size = " << acc_lv2[0].size() << endl;
            cout << "cluster acc_lv2[1] size = " << acc_lv2[1].size() << endl;
            cout << "cluster acc_lv2[2] size = " << acc_lv2[2].size() << endl;
            cout << "cluster acc_lv2[3] size = " << acc_lv2[3].size() << endl;
            
            cout << "**** [" << ns_l2 << "] A.1. get four centroid points of laser" << endl;
            pcl::PointXYZ centroid_lv_0 = calculateClusterCentroid(acc_lv2[0]);
            pcl::PointXYZ centroid_lv_1 = calculateClusterCentroid(acc_lv2[1]);
            pcl::PointXYZ centroid_lv_2 = calculateClusterCentroid(acc_lv2[2]);
            pcl::PointXYZ centroid_lv_3 = calculateClusterCentroid(acc_lv2[3]);
            cout << "centroid_lv_0 = " << centroid_lv_0.x << ", " << centroid_lv_0.y << ", " << centroid_lv_0.z << endl;
            cout << "centroid_lv_1 = " << centroid_lv_1.x << ", " << centroid_lv_1.y << ", " << centroid_lv_1.z << endl;
            cout << "centroid_lv_2 = " << centroid_lv_2.x << ", " << centroid_lv_2.y << ", " << centroid_lv_2.z << endl;
            cout << "centroid_lv_3 = " << centroid_lv_3.x << ", " << centroid_lv_3.y << ", " << centroid_lv_3.z << endl;
            // [four centroids]
            centroid_acc_lv = {centroid_lv_0, centroid_lv_1, centroid_lv_2, centroid_lv_3};
        }
        else
        {
            if(DEBUG) ROS_WARN("[%s] B. Don't use centroid!", ns_l2.c_str());
            centroid_acc_lv = {local_lv[0], local_lv[1], local_lv[2], local_lv[3]};
        }

        if(DEBUG)
            for(vector<pcl::PointXYZ>::iterator it=centroid_acc_lv.begin(); it<centroid_acc_lv.end(); ++it){
                cout << "detected_3d_lidar_wt_centroid" << it - centroid_acc_lv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
            }

        std::vector<double> rmse_3d_lidar_wt_centroid;
    
        if(acc_frame == max_frame)
        {
            laser2_end = true;
            final_centroid_acc_lv2 = centroid_acc_lv;
            ROS_WARN("[%s] REACH THE MAX FRMAE", ns_l2.c_str());
            if(save_final_data && laser1_end && laser2_end)
            {
                ROS_INFO("<<< Saving Data...");
                writeCircleCenters(os_final.str().c_str(), final_centroid_acc_lv1, final_centroid_acc_lv2);           
                writeCircleCenters(os_final_realtime.str().c_str(), final_centroid_acc_lv1, final_centroid_acc_lv2);   

                laser1_end = false;
                laser2_end = false;
                ROS_WARN("****** Final Data Saved!!! (%s) ******", ns_l2.c_str());
                final_saved =true;
                sleep(2);   
            }
        }
    }
    laser2Received = false;
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
        of_final_centers << "time,detected_lv1[0]x,detected_lv1[0]y,detected_lv1[0]z,detected_lv1[1]x,detected_lv1[1]y,detected_lv1[1]z,detected_lv1[2]x,detected_lv1[2]y,detected_lv1[2]z,detected_lv1[3]x,detected_lv1[3]y,detected_lv1[3]z,detected_lv2[0]x,detected_lv2[0]y,detected_lv2[0]z,detected_lv2[1]x,detected_lv2[1]y,detected_lv2[1]z,detected_lv2[2]x,detected_lv2[2]y,detected_lv2[2]z,detected_lv2[3]x,detected_lv2[3]y,detected_lv2[3]z" << endl;
        of_final_centers.close();
        of_final_centers_realtime << "time,detected_lv1[0]x,detected_lv1[0]y,detected_lv1[0]z,detected_lv1[1]x,detected_lv1[1]y,detected_lv1[1]z,detected_lv1[2]x,detected_lv1[2]y,detected_lv1[2]z,detected_lv1[3]x,detected_lv1[3]y,detected_lv1[3]z,detected_lv2[0]x,detected_lv2[0]y,detected_lv2[0]z,detected_lv2[1]x,detected_lv2[1]y,detected_lv2[1]z,detected_lv2[2]x,detected_lv2[2]y,detected_lv2[2]z,detected_lv2[3]x,detected_lv2[3]y,detected_lv2[3]z" << endl;
        of_final_centers_realtime.close();

    }
}

void writeCircleCenters(const char* file_name, vector<pcl::PointXYZ>& centers_v1, vector<pcl::PointXYZ>& centers_v2)
{
    std::ofstream of_file;
    of_file.open(file_name, ios::app);

    of_file << currentDateTime() << "," 
    << centers_v1[0].x << "," << centers_v1[0].y  << "," << centers_v1[0].z  << "," 
    << centers_v1[1].x << "," << centers_v1[1].y << "," << centers_v1[1].z << "," 
    << centers_v1[2].x << "," << centers_v1[2].y << "," << centers_v1[2].z << "," 
    << centers_v1[3].x << "," << centers_v1[3].y << "," << centers_v1[3].z << ","
    << centers_v2[0].x << "," << centers_v2[0].y  << "," << centers_v2[0].z  << "," 
    << centers_v2[1].x << "," << centers_v2[1].y << "," << centers_v2[1].z << "," 
    << centers_v2[2].x << "," << centers_v2[2].y << "," << centers_v2[2].z << "," 
    << centers_v2[3].x << "," << centers_v2[3].y << "," << centers_v2[3].z << endl;   

    of_file.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pattern_collection");
    ros::NodeHandle nh_("~");

    nh_.param<bool>("useCentroid_laser", useCentroid_laser, false);
    nh_.param<bool>("save_final_data", save_final_data, false);
    
    nh_.param<string>("result_dir_", result_dir_, "");
    nh_.param<string>("feature_file_name",feature_file_name_, "");
    nh_.param<string>("ns_l1", ns_l1, "LASER1");
    nh_.param<string>("ns_l2", ns_l2, "LASER2");
    ros::param::get("/max_frame", max_frame);

    laser1Received = false;
    laser1_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    laser2Received = false;
    laser2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    acc_laser1_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    acc_laser2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Subscriber laser1_sub = nh_.subscribe<lvt2calib::ClusterCentroids>("cloud_laser1", 1, laser1_callback);
    ros::Subscriber laser2_sub = nh_.subscribe<lvt2calib::ClusterCentroids>("cloud_laser2", 1, laser2_callback);

    acc_laser1_cloud_pub = nh_.advertise<PointCloud2>("acc_laser1_centers",1);
    acc_laser2_cloud_pub = nh_.advertise<PointCloud2>("acc_laser2_centers",1);

    ROS_INFO("Initialized!");
    int posNo = 0;
    ros::Rate loop_rate(10);

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
        
        while(ros::ok()){
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
                // bool cam_paused = false, livox_paused = false;
                // ros::param::get("/cam_paused", cam_paused);
                // ros::param::get("/livox_paused", livox_paused);
                // while(!cam_paused || !livox_paused)
                // {
                //     ros::param::get("/cam_paused", cam_paused);
                //     ros::param::get("/livox_paused", livox_paused);
                // }
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

                    // ros::param::set("/bag_changed", true);
                    posNo ++;
                    
                    final_saved = false;
                    acc_lv2[0].clear();
                    acc_lv2[1].clear();
                    acc_lv2[2].clear();
                    acc_lv2[3].clear();
                    acc_lv1[0].clear();
                    acc_lv1[1].clear();
                    acc_lv1[2].clear();
                    acc_lv1[3].clear();
                    acc_laser1_cloud->clear();
                    acc_laser2_cloud->clear();  

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