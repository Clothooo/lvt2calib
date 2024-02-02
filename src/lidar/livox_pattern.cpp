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

#include <lvt2calib/ClusterCentroids.h>
#include <lvt2calib/AutoDetectLaser.h>
#include <lvt2calib/FourCircleCenters.h>
#include <lvt2calib/livox_utils.h>
#include <lvt2calib/LaserConfig.h>

#define DEBUG 0

using namespace std;
using namespace sensor_msgs;
using namespace pcl;
using namespace Eigen;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;

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

CloudType::Ptr calib_board_bound_template (new CloudType);
CloudType::Ptr acc_boards(new CloudType);
CloudType::Ptr acc_boards_registed(new CloudType);

ros::Publisher reload_cloud_pub, calib_board_pub, four_center_pub, centers_pub, acc_boards_pub, acc_boards_bound_registed_pub, acc_boards_filterd_pub, plane_segments_pub;
ros::Publisher colored_planes_pub;

string ns_str;

AutoDetectLaser myDetector;
FourCircleCenters myFourCenters;

void load_param(ros::NodeHandle& nh_);
void set_run_param();
void param_callback(lvt2calib::LaserConfig &config, uint32_t level);


void callback(const PointCloud2::ConstPtr& laser_cloud)
{
    ROS_INFO("[%s] Processing cloud...", ns_str.c_str());
    CloudType::Ptr cloud_in (new CloudType),        // Origin Point Cloud
										calib_board (new CloudType),		// calib board pc
                                        calib_board_registed (new CloudType),calib_boundary (new CloudType),
                                        calib_boundary_registed (new CloudType),
                                        acc_calib_boundary_registed (new CloudType),
                                        acc_calib_boundary (new CloudType),
                                        four_circle_centers(new CloudType);		// four circle centers
    
    clouds_proc_++;
    fromROSMsg(*laser_cloud, *cloud_in);
    std_msgs::Header cloud_header = laser_cloud->header;
    
    sensor_msgs::PointCloud2 reload_cloud_ros;
    pcl::toROSMsg(*cloud_in, reload_cloud_ros);
    reload_cloud_ros.header = laser_cloud->header;
    reload_cloud_pub.publish(reload_cloud_ros);         // topic: /livox_pattern/reload_cloud
    
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
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_planes(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool ifDetected = false;
    if(!use_RG_Pseg)
    {
        ifDetected = myDetector.detectCalibBoard(cloud_in, calib_board);
        publishPC<pcl::PointXYZI>(plane_segments_pub, cloud_header, myDetector.colored_i_planes_);
    }
    else
    {
        ifDetected = myDetector.detectCalibBoardRG(cloud_in, calib_board);
        publishPC<pcl::PointXYZRGB>(colored_planes_pub, cloud_header, myDetector.colored_planes_);
    }

    // if(calib_board->points.size() > 0)
    if(ifDetected)
    {
        ROS_WARN("<<<<<< [%s] Have found the calib borad point cloud!!!", ns_str.c_str());
        publishPC<PointType>(calib_board_pub, cloud_header, calib_board);    // topic: /livox_pattern/calib_board_cloud

        // ROS_WARN("<<<<<<< calib_board->points.size() > 0");
       
        // if(doAccBoards && (acc_board_num < max_acc_frame_))
        if(doAccBoards)
        {
            acc_board_num++;
            ROS_WARN("<<<<<< [%s] Accumalated %d frames of board cloud..........", ns_str.c_str(), acc_board_num);
            *acc_boards += *calib_board;
            
            CloudType::Ptr voxel_filtered(new CloudType);
            pcl::VoxelGrid<PointType> sor;
            sor.setInputCloud(acc_boards);
            sor.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
            sor.filter(*voxel_filtered);
            pcl::copyPointCloud(*voxel_filtered, *acc_boards);

            CloudType::Ptr cloud_gauss_filtered (new CloudType);
            pcl::copyPointCloud(*acc_boards, *cloud_gauss_filtered);
            if(use_gauss_filter2_)
            {
                // filters for accumulated board point cloud
                // **************** 基于高斯核函数的卷积滤波实现 *****************
                pcl::filters::GaussianKernel<PointType, PointType> kernel;
                kernel.setSigma(gauss_k_sigma2_);    // 高斯函数的标准方差，决定函数的宽度
                kernel.setThresholdRelativeToSigma(gauss_k_thre_rt_sigma2_); //　设置相对sigma参数的距离阈值
                kernel.setThreshold(gauss_k_thre2_); //　设置距离阈值，若点间距离大于阈值则不予考虑
                // cout << "Kernel made" << endl;

                pcl::search::KdTree<PointType>::Ptr gauss_tree(new pcl::search::KdTree<PointType>);
                gauss_tree->setInputCloud(acc_boards);
                // cout << "KdTree made" << endl;
                
                // *************** 设置Convolution相关参数 *****************
                pcl::filters::Convolution3D<PointType, PointType, pcl::filters::GaussianKernel<PointType, PointType>> convolution;
                convolution.setKernel(kernel); // 设置卷积核
                convolution.setInputCloud(acc_boards);
                convolution.setNumberOfThreads(8);
                convolution.setSearchMethod(gauss_tree);
                convolution.setRadiusSearch(gauss_conv_radius2_);
                // cout << "Convolution start" << endl;

                convolution.convolve(*cloud_gauss_filtered);
                // cout << "cluster size after gauss filter: " << cloud_gauss_filtered->points.size() << endl;
                // if(auto_mode_) showPointXYZI(cloud_gauss_filtered, 1, "after gauss filter");
                // pcl::copyPointCloud(*cloud_gauss_filtered, *acc_boards);
                
                CloudType::Ptr voxel2_filtered(new CloudType);
                pcl::VoxelGrid<PointType> sor2;
                sor2.setInputCloud(cloud_gauss_filtered);
                sor2.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
                sor2.filter(*voxel2_filtered);
                pcl::copyPointCloud(*voxel2_filtered, *cloud_gauss_filtered);

                CloudType::Ptr cloud_s_filtered(new CloudType);
                pcl::StatisticalOutlierRemoval<PointType> sor;
                sor.setInputCloud(cloud_gauss_filtered);
                sor.setMeanK(sor_MeanK_);
                sor.setStddevMulThresh(sor_StddevMulThresh_);
                sor.filter(*cloud_s_filtered);
                pcl::copyPointCloud(*cloud_s_filtered, *cloud_gauss_filtered);

                sensor_msgs::PointCloud2 acc_boards_filterd_ros;
                pcl::toROSMsg(*cloud_gauss_filtered, acc_boards_filterd_ros);
                acc_boards_filterd_ros.header = laser_cloud->header;
                acc_boards_filterd_pub.publish(acc_boards_filterd_ros);	// topic: /livox_pattern/acc_boards_filtered
            }

            sensor_msgs::PointCloud2 acc_boards_ros;
            pcl::toROSMsg(*acc_boards, acc_boards_ros);
            acc_boards_ros.header = laser_cloud->header;
            acc_boards_pub.publish(acc_boards_ros);		// topic: /livox_pattern/acc_boards

            // myDetector.isCalibBoard(acc_boards, calib_board_bound_template, acc_calib_boundary_registed);
            if(myDetector.isCalibBoard(cloud_gauss_filtered, acc_calib_boundary, acc_calib_boundary_registed))
            {
                Eigen::Matrix4f Tr_calib2tpl = myDetector.Tr_ukn2tpl_;  

                sensor_msgs::PointCloud2 acc_boards_bound_registed_ros;
                pcl::toROSMsg(*acc_calib_boundary_registed, acc_boards_bound_registed_ros);
                acc_boards_bound_registed_ros.header = laser_cloud->header;
                acc_boards_bound_registed_pub.publish(acc_boards_bound_registed_ros);	// topic: /livox_pattern/acc_boards_bound_registed

                bool find_centers = myFourCenters.FindFourCenters(calib_board_bound_template, four_circle_centers, Tr_calib2tpl.inverse());
                // bool find_centers = myFourCenters.FindFourCenters(acc_calib_boundary, four_circle_centers, Eigen::Matrix4f::Identity());

                if(find_centers) board_used_num++;
                ROS_WARN("<<<<<< [%s] board used num = %d", ns_str.c_str(), board_used_num);

                sensor_msgs::PointCloud2 four_center_ros;
                pcl::toROSMsg(*four_circle_centers, four_center_ros);
                four_center_ros.header = laser_cloud->header;
                four_center_pub.publish(four_center_ros);			// topic: /livox_pattern/four_center

                lvt2calib::ClusterCentroids to_send;
                to_send.header = laser_cloud->header;
                // to_send.cluster_iterations = acc_board_num;
                to_send.cluster_iterations = board_used_num;
                to_send.total_iterations = acc_board_num;
                to_send.cloud = four_center_ros;
                centers_pub.publish(to_send);       // Topic: /livox_pattern/centers_tosend

                if(DEBUG) ROS_INFO("Pattern centers published");
            }

        }
        // else if(doAccBoards)
        // {
        //     ROS_WARN("<<<<<< [%s] REACHED THE MAX ACC FRAME!!!", ns_str.c_str());
        //     // ros::shutdown();
        // }
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
    // cout << "max_size = " << max_size << endl;
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
    ros::init(argc, argv, "livox_pattern");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    // ros::Rate loop_rate(1);

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

    reload_cloud_pub = nh_.advertise<PointCloud2>("cloud_in", 1);
    plane_segments_pub = nh_.advertise<PointCloud2>("plane_segments", 1);
	calib_board_pub = nh_.advertise<PointCloud2>("calib_board_cloud", 1);
    four_center_pub = nh.advertise<PointCloud2>("/" + ns_str + "/laser_pattern_circle/circle_center_cloud", 1);
    acc_boards_pub = nh_.advertise<PointCloud2>("acc_boards", 1);
    acc_boards_bound_registed_pub = nh_.advertise<PointCloud2>("acc_boards_bound_registed", 1);
    acc_boards_filterd_pub = nh_.advertise<PointCloud2>("acc_boards_filtered", 1);
    centers_pub = nh_.advertise<lvt2calib::ClusterCentroids>("/"+ns_str+"/centers_cloud", 1);
    
    colored_planes_pub = nh_.advertise<PointCloud2>("colored_planes", 1);

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