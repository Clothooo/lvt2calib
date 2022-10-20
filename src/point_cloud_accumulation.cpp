// ********* point_cloud_accumulation
// time integration of point cloud from LiDAR
// by yao

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>

#include <iomanip>
#include <unistd.h>
#include <stdio.h>

#include <lvt2calib/PcAccConfig.h>

#define DEBUG 0

using namespace pcl;
using namespace std;
using namespace sensor_msgs;
using namespace ros;

ros::Publisher acc_pc_pub_, acc_pc_pub2_;

int cloud_frame_ = 0, acc_num_ = 1, pre_acc_num_;
std::string cloud_pc_tp, cloud_pc2_tp;
pcl::PointCloud<pcl::PointXYZI>::Ptr acc_pc(new pcl::PointCloud<pcl::PointXYZI>);
vector<size_t> pc_size_v;

void callback_pc2(const sensor_msgs::PointCloud2::ConstPtr& laser_cloud2)
{
    if(DEBUG)
        ROS_INFO("GET PC2 MSG!");
    pcl::PointCloud<PointXYZI>::Ptr cloud_in(new pcl::PointCloud<PointXYZI>);
    fromROSMsg(*laser_cloud2, *cloud_in);
   

    if(acc_num_ != pre_acc_num_ && pc_size_v.size() > acc_num_)
    {   
        int num_gap = pc_size_v.size() - acc_num_;
        size_t total_size_erase = 0;
        for(int i = 0; i < num_gap; i++)
        {
            total_size_erase += pc_size_v[i];
        }
        acc_pc->points.erase(acc_pc->points.begin(), acc_pc->points.begin()+total_size_erase);
        pc_size_v.erase(pc_size_v.begin(), pc_size_v.begin()+num_gap);
    }

    if(pc_size_v.size() >= acc_num_)
    {
        size_t front_pc_size = pc_size_v.front();
        pc_size_v.erase(pc_size_v.begin());
        acc_pc->points.erase(acc_pc->points.begin(), acc_pc->points.begin()+front_pc_size);
    }

    *acc_pc += *cloud_in;
    pc_size_v.push_back(cloud_in->points.size());
  
    if(pc_size_v.size() == acc_num_)
    {
        if(DEBUG)
            ROS_INFO("<<<<< Accumulate %d frames point cloud", acc_num_);

        sensor_msgs::PointCloud2 acc_pc_ros;
        pcl::toROSMsg(*acc_pc, acc_pc_ros);
        acc_pc_ros.header = laser_cloud2->header;
        acc_pc_pub2_.publish(acc_pc_ros); 
    }
}

void callback_pc(const sensor_msgs::PointCloud::ConstPtr& laser_cloud)
{
    if(DEBUG)
        ROS_INFO("GET PC1 MSG!");
    PointCloud2Ptr laser_cloud2(new PointCloud2);
    convertPointCloudToPointCloud2(*laser_cloud, *laser_cloud2);

    pcl::PointCloud<PointXYZI>::Ptr cloud_in(new pcl::PointCloud<PointXYZI>);
    fromROSMsg(*laser_cloud2, *cloud_in);
    
     if(acc_num_ != pre_acc_num_ && pc_size_v.size() > acc_num_)
    {   
        int num_gap = pc_size_v.size() - acc_num_;
        size_t total_size_erase = 0;
        for(int i = 0; i < num_gap; i++)
        {
            total_size_erase += pc_size_v[i];
        }
        acc_pc->points.erase(acc_pc->points.begin(), acc_pc->points.begin()+total_size_erase);
        pc_size_v.erase(pc_size_v.begin(), pc_size_v.begin()+num_gap);
    }

    if(pc_size_v.size() >= acc_num_)
    {
        size_t front_pc_size = pc_size_v.front();
        pc_size_v.erase(pc_size_v.begin());
        acc_pc->points.erase(acc_pc->points.begin(), acc_pc->points.begin()+front_pc_size);
    }

    *acc_pc += *cloud_in;
    pc_size_v.push_back(cloud_in->points.size());
  
    if(pc_size_v.size() == acc_num_)
    {
        if(DEBUG)
            ROS_INFO("<<<<< Accumulate %d frames point cloud", acc_num_);

        sensor_msgs::PointCloud2 acc_pc_ros;
        pcl::toROSMsg(*acc_pc, acc_pc_ros);
        acc_pc_ros.header = laser_cloud2->header;
        acc_pc_pub_.publish(acc_pc_ros); 
    }
}

void param_callback(lvt2calib::PcAccConfig &config, uint32_t level)
{
    pre_acc_num_ = acc_num_;
    acc_num_ = config.acc_num;
    ROS_INFO("New number of accumulate frames: %d", acc_num_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_accumulation");
    ros::NodeHandle nh_("~");

    bool if_pc_in = false, if_pc2_in = true;
    // nh_.getParam("cloud_in_pc2", cloud_pc2_tp);
    // ROS_INFO("get param 'cloud_in_pc2': %s", cloud_pc2_tp.c_str());

    if(ros::param::get("~cloud_in_pc2", cloud_pc2_tp))
    {
        ROS_INFO("get param 'cloud_in_pc2': %s", cloud_pc2_tp.c_str());
        if_pc2_in = true;
    }
    if(ros::param::get("~cloud_in_pc", cloud_pc_tp))
    {
        ROS_INFO("get param 'cloud_in_pc': %s", cloud_pc_tp.c_str());
        if_pc_in = true;
    }


    ros::Subscriber sub1 = nh_.subscribe(cloud_pc_tp, 5, callback_pc); //for sensor_msgs/PointCloud
    ros::Subscriber sub2 = nh_.subscribe(cloud_pc2_tp, 5, callback_pc2); //for sensor_msgs/PointCloud2
    pc_size_v.resize(acc_num_); 

    if(if_pc_in)
        acc_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_pc_tp+"/acc_cloud", 1);
    if(if_pc2_in)
        acc_pc_pub2_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_pc2_tp+"/acc_cloud", 1);

    dynamic_reconfigure::Server<lvt2calib::PcAccConfig> server;
    dynamic_reconfigure::Server<lvt2calib::PcAccConfig>::CallbackType f;
    f = boost::bind(param_callback, _1, _2);
    server.setCallback(f);

    cout << "initialized........." << endl;

    while (ros::ok())
    {
        ros::spin();
    }
    ros::shutdown();
    

    return 0;
}