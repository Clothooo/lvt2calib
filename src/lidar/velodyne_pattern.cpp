#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/console/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <iomanip>
#include <boost/thread/thread.hpp>
#include <thread>

#include <lvt2calib/AutoDetectLaser.h>
#include <lvt2calib/FourCircleCenters.h>
#include <lvt2calib/velo_utils.h>
#include <lvt2calib/LaserConfig.h>

#define DEBUG 0

using namespace std;
using namespace sensor_msgs;
using namespace pcl;
using namespace Eigen;

typedef Velodyne::Point PointType;
typedef pcl::PointCloud<PointType> CloudType;
FIX_LASER_TYPE laser_type = VELO_16;

int queue_size_ = 1;
bool pos_changed_ = false;

bool use_RG_Pseg = false;
bool use_vox_filter_ = true, use_i_filter_ = true,
     use_gauss_filter_, auto_mode_ = true, use_statistic_filter_,
     is_gazebo = false, use_gauss_filter2_;
double re, reforn, Pseg_size_min_,
        remove_x_min_, remove_x_max_,
        cluster_tole_, cluster_size_min_, cluster_size_max_, 
        i_filter_out_min_, i_filter_out_max_,
        rmse_ukn2tpl_thre_, rmse_tpl2ukn_thre_,
        circle_seg_thre_, circle_radius_,
        centroid_dis_min_, centroid_dis_max_,
        voxel_grid_size_, Pseg_dis_thre_,
        RG_smooth_thre_deg_, RG_curve_thre_;
double gauss_k_sigma_, gauss_k_thre_rt_sigma_, gauss_k_thre_,
        gauss_conv_radius_;
double gauss_k_sigma2_, gauss_k_thre_rt_sigma2_, gauss_k_thre2_,
        gauss_conv_radius2_;
int Pseg_iter_num_, min_centers_found_, max_acc_frame_ = 0, 
    sor_MeanK_, sor_StddevMulThresh_, RG_neighbor_n_;
std::string model_path = "";

double Rad_to_deg = 45.0 / atan(1.0), 
        max_size = 120 * 80;
int clouds_proc_ = 0, clouds_used_ = 0;
int acc_board_num = 0, board_used_num = 0;
bool doAccBoards = false;
string ns_str;

pcl::PointCloud<pcl::PointXYZI>::Ptr calib_board_bound_template (new pcl::PointCloud<pcl::PointXYZI>);
CloudType::Ptr acc_boards(new CloudType);
CloudType::Ptr acc_boards_registed(new CloudType);

ros::Publisher reload_cloud_pub, calib_board_pub;
ros::Publisher cloud_in_pub, colored_i_planes_pub, icp_regist_boundary_pub, template_pc_pub, raw_boundary_pub, colored_planes_pub;

AutoDetectLaser myDetector(R_LIDAR);
FourCircleCenters myFourCenters;

void load_param(ros::NodeHandle& nh_);
void set_run_param();
void param_callback(lvt2calib::LaserConfig &config, uint32_t level);


void callback(const PointCloud2::ConstPtr& laser_cloud)
{
    ROS_INFO("[%s] Processing cloud...", ns_str.c_str());
    std_msgs::Header cloud_header = laser_cloud->header;
    CloudType::Ptr cloud_in (new CloudType),        // Origin Point Cloud
                    cloud_reload (new CloudType);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_copy (new pcl::PointCloud<pcl::PointXYZI>), 
                                        calib_board (new pcl::PointCloud<pcl::PointXYZI>); // calib board pc
    clouds_proc_++;
    fromROSMsg(*laser_cloud, *cloud_in);

    Velodyne::addRange(*cloud_in);
    // Velodyne::normalizeIntensity(*cloud_in, 0, 255);
    for(auto it = cloud_in->points.begin(); it < cloud_in->points.end(); it++)
        it->intensity  = 0;
    publishPC<PointType>(cloud_in_pub, cloud_header, cloud_in);

    pcl::copyPointCloud(*cloud_in, *cloud_in_copy);
    Velodyne::resetIntensity(*cloud_in);
    pcl::copyPointCloud(*cloud_in, *cloud_reload);


    if(pos_changed_)
    {
        if(clouds_proc_ <= queue_size_)
            return;
        else
        {
            pos_changed_ = false;
            // ROS_WARN("****** queue clear, clouds_proc_ = %d ******", clouds_proc_);
        }
    }

	bool ifDetected = false;
    if(!use_RG_Pseg)
    {
        ifDetected = myDetector.detectCalibBoard(cloud_in_copy, calib_board);
        publishPC<pcl::PointXYZI>(colored_i_planes_pub, cloud_header, myDetector.colored_i_planes_);
    }    
    else
    {
        ifDetected = myDetector.detectCalibBoardRG(cloud_in_copy, calib_board);
        publishPC<pcl::PointXYZRGB>(colored_planes_pub, cloud_header, myDetector.colored_planes_);
    }

    publishPC<pcl::PointXYZI>(icp_regist_boundary_pub, cloud_header, myDetector.calib_board_boundary_registed_);
    publishPC<pcl::PointXYZI>(template_pc_pub, cloud_header, calib_board_bound_template);
    publishPC<pcl::PointXYZI>(raw_boundary_pub, cloud_header, myDetector.calib_board_boundary_);
    
    // if(calib_board->points.size() > 0)
    if(ifDetected)
    {
        ROS_WARN("<<<<<< [%s] Have found the calib borad point cloud!!!", ns_str.c_str());
        publishPC<pcl::PointXYZI>(calib_board_pub, cloud_header, calib_board);   // topic: /velodyne_pattern/calib_board_cloud
        publishPC<PointType>(reload_cloud_pub, cloud_header, cloud_reload);
    }
    else{
        ROS_WARN("<<<<<< [%s] CANNOT find the calib borad!", ns_str.c_str());
    }
}

void param_callback(lvt2calib::LaserConfig &config, uint32_t level)
{
    remove_x_min_ = config.remove_x_min;
    remove_x_max_ = config.remove_x_max;
    ROS_INFO("New remove_x_min_: %f ", remove_x_min_);
    ROS_INFO("New remove_x_max_: %f ", remove_x_max_);
    
    voxel_grid_size_ = config.voxel_grid_size;
    ROS_INFO("New voxel_grid_size_: %f ", voxel_grid_size_);
    
    gauss_k_sigma_ = config.gauss_k_sigma;
    gauss_k_thre_rt_sigma_ = config.gauss_k_thre_rt_sigma;
    gauss_k_thre_ = config.gauss_k_thre;
    gauss_conv_radius_ = config.gauss_conv_radius;
    ROS_INFO("New gauss_k_sigma_: %f ", gauss_k_sigma_);
    ROS_INFO("New gauss_k_thre_rt_sigma_: %f ", gauss_k_thre_rt_sigma_);
    ROS_INFO("New gauss_k_thre_: %f ", gauss_k_thre_);
    ROS_INFO("New gauss_conv_radius_: %f ", gauss_conv_radius_);

    cluster_tole_ = config.cluster_tole;
    cluster_size_min_ = config.cluster_size_min;
    cluster_size_max_ = config.cluster_size_max;
    ROS_INFO("New cluster_tole_: %f ", cluster_tole_);
    ROS_INFO("New cluster_size_min_: %f ", cluster_size_min_);
    ROS_INFO("New cluster_size_max_: %f ", cluster_size_max_);

    Pseg_dis_thre_ = config.Pseg_dis_thre;
    Pseg_iter_num_ = config.Pseg_iter_num;
    Pseg_size_min_ = config.Pseg_size_min;
    ROS_INFO("New Pseg_dis_thre_: %f ", Pseg_dis_thre_);
    ROS_INFO("New Pseg_iter_num_: %d ", Pseg_iter_num_);
    ROS_INFO("New Pseg_size_min_: %f ", Pseg_size_min_);

    RG_smooth_thre_deg_ = config.RG_smooth_thre_deg;
    RG_curve_thre_ = config.RG_curve_thre;
    RG_neighbor_n_ = config.RG_neighbor_n;
    ROS_INFO("New RG_smooth_thre_deg_: %f ", RG_smooth_thre_deg_);
    ROS_INFO("New RG_curve_thre_: %f ", RG_curve_thre_);
    ROS_INFO("New RG_neighbor_n_: %d ", RG_neighbor_n_);

    sor_MeanK_ = config.sor_MeanK;
    sor_StddevMulThresh_ = config.sor_StddevMulThresh;
    ROS_INFO("New sor_MeanK_: %d ", sor_MeanK_);
    ROS_INFO("New sor_StddevMulThresh_: %d ", sor_StddevMulThresh_);
    
    i_filter_out_min_ = config.i_filter_out_min;
    i_filter_out_max_ = config.i_filter_out_max;
     // In simulation, cancel the intensity filter.
    if(is_gazebo)
    {
        i_filter_out_min_ = 0;
        i_filter_out_max_ = 0;
    }
    ROS_INFO("New i_filter_out_min_: %f ", i_filter_out_min_);
    ROS_INFO("New i_filter_out_max_: %f ", i_filter_out_max_);

    re = config.boundEstRad;
    reforn = config.normEstRad;
    ROS_INFO("New boundEstRad: %f ", re);
    ROS_INFO("New normEstRad: %f ", reforn);

    rmse_ukn2tpl_thre_ = config.rmse_ukn2tpl_thre;
    rmse_tpl2ukn_thre_ = config.rmse_tpl2ukn_thre;
    ROS_INFO("New rmse_ukn2tpl_thre_: %f ", rmse_ukn2tpl_thre_);
    ROS_INFO("New rmse_tpl2ukn_thre_: %f ", rmse_tpl2ukn_thre_);

    circle_radius_ = config.circle_radius;
    circle_seg_thre_ = config.circle_seg_thre;
    centroid_dis_min_ = config.centroid_dis_min;
    centroid_dis_max_ = config.centroid_dis_max;
    min_centers_found_ = config.min_centers_found;
    ROS_INFO("New circle_radius_: %f ", circle_radius_);
    ROS_INFO("New circle_seg_thre_: %f ", circle_seg_thre_);
    ROS_INFO("New centroid_dis_min_: %f ", centroid_dis_min_);
    ROS_INFO("New centroid_dis_max_: %f ", centroid_dis_max_);
    ROS_INFO("New min_centers_found_: %d ", min_centers_found_);
    // the theoretical max size of the point cloud on the calirbation plane
    max_size = (1.2 * 0.8 - 4 * M_PI * pow(circle_radius_, 2)) * 2.0 * Pseg_dis_thre_ / pow(voxel_grid_size_, 3);
    // double max_size = (1.2 * 0.8 - 4 * M_PI * pow(circle_radius_, 2)) / (voxel_grid_size_ * voxel_grid_size_);
    ROS_INFO("New max_size: %f ", max_size);

    gauss_k_sigma2_ = config.gauss_k_sigma2;
    gauss_k_thre_rt_sigma2_ = config.gauss_k_thre_rt_sigma2;
    gauss_k_thre2_ = config.gauss_k_thre2;
    gauss_conv_radius2_ = config.gauss_conv_radius2;
    ROS_INFO("New gauss_k_sigma2_: %f ", gauss_k_sigma2_);
    ROS_INFO("New gauss_k_thre_rt_sigma2_: %f ", gauss_k_thre_rt_sigma2_);
    ROS_INFO("New gauss_k_thre2_: %f ", gauss_k_thre2_);
    ROS_INFO("New gauss_conv_radius2_: %f ", gauss_conv_radius2_);

    set_run_param();
}

void load_param(ros::NodeHandle& nh_)
{
    nh_.param<std::string>("model_path", model_path, "");
    nh_.param("is_gazebo", is_gazebo, false);
    nh_.param("use_vox_filter", use_vox_filter_, true);
    nh_.param("use_i_filter", use_i_filter_, true);
    nh_.param("use_gauss_filter", use_gauss_filter_, true);
    nh_.param("use_gauss_filter2", use_gauss_filter2_, true);
    nh_.param("use_statistic_filter", use_statistic_filter_, false);
    nh_.param("use_RG_Pseg", use_RG_Pseg, false);
    nh_.param("queue_size", queue_size_, 1);
    nh_.param<std::string>("ns", ns_str, "laser");

    return;
}

void set_run_param()
{
    // ***************** set auto_detect_calirbation_pattern param
    myDetector.setRemoveRangeX(remove_x_min_, remove_x_max_);
    myDetector.useVoxelFilter(use_vox_filter_);
    myDetector.setVoxelFilterSize(voxel_grid_size_);
    myDetector.setAutoMode(auto_mode_);     // auto_mode = true for applictaion; auto_mode = false for DEBUG, show results of every step (no realization).
    myDetector.useGaussFilter(use_gauss_filter_);
    myDetector.setGaussFilterParam(gauss_k_sigma_, gauss_k_thre_rt_sigma_, gauss_k_thre_, gauss_conv_radius_);
    myDetector.useStatisticalFilter(use_statistic_filter_);
    myDetector.setStatisticalFilterParam(sor_MeanK_, sor_StddevMulThresh_);

    myDetector.setClusterParam(cluster_tole_, cluster_size_min_ * max_size, cluster_size_max_ * max_size);
    myDetector.setPlaneSegmentationParam(Pseg_dis_thre_, Pseg_size_min_, Pseg_iter_num_);
    myDetector.setRGPlaneSegmentationParam(RG_smooth_thre_deg_, RG_curve_thre_, RG_neighbor_n_);
    myDetector.useIntensityFilter(use_i_filter_);
    myDetector.setIntensityFilterParam(i_filter_out_min_, i_filter_out_max_);

    myDetector.setBoundEstKSearch(re);
    myDetector.setNormEstKSearch(reforn);
    myDetector.setDiffRMSEThreshold(rmse_ukn2tpl_thre_, rmse_tpl2ukn_thre_);
    
    // ***************** set four_circle_centers param
    myFourCenters.setCircleSegDistanceThreshold(circle_seg_thre_);
    myFourCenters.setCircleRadius(circle_radius_);
    myFourCenters.setCentroidDis(centroid_dis_min_, centroid_dis_max_);
    myFourCenters.setMinNumCentersFound(min_centers_found_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_pattern");
    ros::NodeHandle nh_("~");
    // ros::Rate loop_rate(1);
    // ns_str = nh_.getNamespace();
    // ns_str = nh_.getUnresolvedNamespace();

    // ***************** load param
    load_param(nh_);
    // set_run_param();

	ostringstream os;
	os << model_path;
	
    // ***************** load template point cloud
	if(DEBUG)
    {
        cout << "loading file" << os.str() << endl;
    }
	pcl::io::loadPCDFile(os.str(), *calib_board_bound_template);
    myDetector.setCalibTemplate(*calib_board_bound_template);

    ros::Subscriber sub = nh_.subscribe("cloud_laser", queue_size_, callback);

    reload_cloud_pub = nh_.advertise<PointCloud2>("reload_cloud", 1);
	calib_board_pub = nh_.advertise<PointCloud2>("calib_board_cloud", 1);

    cloud_in_pub = nh_.advertise<PointCloud2>("cloud_in", 1);
	colored_i_planes_pub = nh_.advertise<PointCloud2>("plane_segments", 1);
	icp_regist_boundary_pub = nh_.advertise<PointCloud2>("icp_regist_boundary", 1);
	template_pc_pub = nh_.advertise<PointCloud2>("template_pc", 1);
    raw_boundary_pub = nh_.advertise<PointCloud2>("raw_boundary_pc", 1);
    colored_planes_pub = nh_.advertise<PointCloud2>("colored_planes_pc", 1);
    
    bool end_process = false;
    bool pause_process = false;
    ros::param::set("/livox_paused", false);

    dynamic_reconfigure::Server<lvt2calib::LaserConfig> server;
    dynamic_reconfigure::Server<lvt2calib::LaserConfig>::CallbackType f;
    f = boost::bind(param_callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::param::get("/end_process", end_process);
        ros::param::get("/pause_process", pause_process);
        ros::param::get("/max_frame", max_acc_frame_);
        ros::param::get("/do_acc_boards", doAccBoards);
        if(end_process)
                break;
        if(pause_process)
        {
            ROS_WARN("<<<<<<<<<<<<<<<<<<< [%s] PAUSE <<<<<<<<<<<<<<<<<<<", ns_str.c_str());
            ros::param::set("/livox_paused", true);
            acc_board_num = 0;
            acc_boards->clear();
            acc_boards_registed->clear();

            board_used_num = 0;
            clouds_proc_ = 0;
            clouds_used_ = 0;
            while(pause_process && ros::ok())
            {
                ros::param::get("/end_process", end_process);
                ros::param::get("/pause_process", pause_process);
                if(end_process)
                    break;
            }
            if(end_process)
                break;
            ros::param::set("/livox_paused", false);
            pos_changed_ = true;
        }
        else
        {
            ros::spinOnce();
		    loop_rate.sleep();
        }
    }

    ROS_WARN("<<<<<<<<<<<<<<<<<<< [%s] END <<<<<<<<<<<<<<<<<<<", ns_str.c_str());
    ros::shutdown();
    return 0;    
}