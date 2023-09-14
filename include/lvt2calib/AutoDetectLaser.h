#ifndef AutoDetectLaser_H
#define AutoDetectLaser_H

#define PCL_NO_RECOMPILE
#define DEBUG 0
#define DEBUG2 0
#define DEBUG3 0
// #define STATIC_ANALYSE 1

#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/console/time.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/shot_omp.h>
#include "pcl/features/fpfh.h"
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>


#include "PCAestimation.h"
// #include "diff_Eigenvalues.h"
// #include "estimateBorders.h"
#include "FourCircleCenters.h"
#include "EstimateBoundary.h"

#ifdef STATIC_ANALYSE
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#endif


using namespace std;
using namespace Eigen;
using namespace pcl;

typedef pcl::PointXYZI PointType_;
typedef pcl::PointCloud<PointType_> CloudType_;

enum LASER_TYPE { NR_LIDAR = 0, R_LIDAR };

int getRandomNumber();
template <typename PointT>
void publishPC(const ros::Publisher& pc_pub_, const std_msgs::Header& header_, typename pcl::PointCloud<PointT>::Ptr& cloud_);

class AutoDetectLaser: public EstimateBoundary<PointType_>
{
    private:
        double rmse_ukn2tpl_thre_, rmse_tpl2ukn_thre_;
        bool use_vox_filter_ = true;
        bool auto_mode_ = false;
        bool use_gauss_filter_ = true;  // for noisy environment
        bool use_i_filter_ = true;
        bool use_statistic_filter_ = false; // for noisy environment

        double voxel_grid_size_ = 0.01;
        double gauss_k_sigma_ = 4, gauss_k_thre_rt_sigma_ = 4, gauss_k_thre_ = 0.05, gauss_conv_radius_ = 0.05; 
        double cluster_tole_ = 0.05, cluster_size_min_ = 30, cluster_size_max_ = 1000000;
        double Pseg_dis_thre_ = 0.01, Pseg_size_min_ = 0.1;
        int Pseg_iter_num_ = 1000;
        double i_filter_out_min_ = 0, i_filter_out_max_ = 15;
        int sor_MeanK_ = 20, sor_StddevMulThresh_ = 1;

        double RG_smooth_thre_deg_ = 10.0, RG_curve_thre_ = 0.1;
        int RG_neighbor_n_ = 30;

        pcl::PointCloud<pcl::PointXYZI>::Ptr calib_template_;

    public:
        LASER_TYPE laser_type_ = NR_LIDAR;

        // for PCA estimation
        Eigen::Vector4f C_source, C_target;             // Centroid
        Eigen::Matrix3f U_source, U_target;             // Eiegnvectors
        Eigen::Vector3f lamda_source, lamda_target;     // Eiegnvalues

        // for auto-detect
        Eigen::Matrix4f Tr_ukn2tpl_, Tr_calib2tpl_;
        double rmse_ukn2tpl_, rmse_tpl2ukn_, icp_score_ = 0.0;
        double remove_x_min_ = -1.0, remove_x_max_ = 1.0;
        std::vector<double> Pseg_time_v, check_time_v;

        // for debug view
        CloudType_::Ptr x_filtered_, ds_filtered_, gs_filtered_, pca_regist_pc_, pca_regist_boundary_, icp_regist_boundary_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr colored_i_planes_;

        CloudType_::Ptr calib_board_boundary_, calib_board_boundary_registed_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_planes_;

        AutoDetectLaser(LASER_TYPE type_ = NR_LIDAR){
            x_filtered_ = CloudType_::Ptr (new CloudType_);
            ds_filtered_ = CloudType_::Ptr (new CloudType_);
            gs_filtered_ = CloudType_::Ptr (new CloudType_);
            pca_regist_pc_ = CloudType_::Ptr (new CloudType_);
            pca_regist_boundary_ = CloudType_::Ptr (new CloudType_);
            icp_regist_boundary_ = CloudType_::Ptr (new CloudType_);

            calib_template_ = CloudType_::Ptr (new CloudType_);
            colored_i_planes_ = CloudType_::Ptr (new CloudType_);
            calib_board_boundary_ = CloudType_::Ptr (new CloudType_);           
            calib_board_boundary_registed_ = CloudType_::Ptr (new CloudType_);

            colored_planes_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        };
        ~AutoDetectLaser(){};

        void setCalibTemplate(pcl::PointCloud<pcl::PointXYZI>& template_pc_)
        {
            *calib_template_ = template_pc_;
        }
        void setRemoveRangeX(double min_, double max_)
        {
            remove_x_min_ = min_;
            remove_x_max_ = max_;
        }
        void setDiffRMSEThreshold(double ukn2tpl, double tpl2ukn)
        {
            rmse_ukn2tpl_thre_ = ukn2tpl;
            rmse_tpl2ukn_thre_ = tpl2ukn;
        }
        void useVoxelFilter(bool flag)
        {
            use_vox_filter_ = flag;
        }
        void setVoxelFilterSize(double x) { voxel_grid_size_ = x;}
        void setAutoMode(bool flag) { auto_mode_ = flag;}
        void useGaussFilter(bool flag) { use_gauss_filter_ = flag;}
        void setGaussFilterParam(double kernel_Sigma, double kernel_ThresholdRelativeToSigma, double kernel_Threshold, double conv_RadiusSearch)
        {
            gauss_k_sigma_ = kernel_Sigma;
            gauss_k_thre_rt_sigma_ = kernel_ThresholdRelativeToSigma;
            gauss_k_thre_ = kernel_Threshold;
            gauss_conv_radius_ = conv_RadiusSearch;
        }
        void setClusterParam(double cluster_Tolerance, double cluster_MinSize, double cluster_MaxSize)
        {
            cluster_tole_ = cluster_Tolerance;
            cluster_size_min_ = cluster_MinSize;
            cluster_size_max_ = cluster_MaxSize;
        }
        void useStatisticalFilter(bool flag) { use_statistic_filter_ = flag; }
        void setStatisticalFilterParam(int MeanK, int StddevMulThresh)
        {
            sor_MeanK_ = MeanK;
            sor_StddevMulThresh_ = StddevMulThresh;
        }
        void setPlaneSegmentationParam(double seg_DistanceThreshold, double min_PlaneSize, int seg_MaxIterations)
        {
            Pseg_dis_thre_ = seg_DistanceThreshold;
            Pseg_size_min_ = min_PlaneSize;
            Pseg_iter_num_ = seg_MaxIterations;
        }
        void useIntensityFilter(bool flag) { use_i_filter_ = flag; }
        void setIntensityFilterParam(double min_RemoveIntensity, double max_RemoveIntensity)
        {
            i_filter_out_min_ = min_RemoveIntensity;
            i_filter_out_max_ = max_RemoveIntensity;
        }
        void setRGPlaneSegmentationParam(double rg_smooth_thre_deg_, double rg_curve_thre_, int n_){
            RG_smooth_thre_deg_ = rg_smooth_thre_deg_;
            RG_curve_thre_ = rg_curve_thre_;
            RG_neighbor_n_ = n_;
        }

        bool isCalibBoard(CloudType_::Ptr& cloud, 
                CloudType_::Ptr& cloud_boundary,
                CloudType_::Ptr& cloud_boundary_registed);
        Eigen::Matrix4f PCARegistration(CloudType_::Ptr& source_cloud, CloudType_::Ptr& target_cloud);
        float CalculateRMSE(std::vector<float> data);
        float ComputeDifference(CloudType_::Ptr& source, CloudType_::Ptr& target);
        void RemoveFloor(CloudType_::Ptr& cloud_in, CloudType_::Ptr& cloud_out, float part);
        bool detectCalibBoard(CloudType_::Ptr &cloud_in, 
                                        CloudType_::Ptr &calib_board);
        bool detectCalibBoardRG(CloudType_::Ptr &cloud_in, 
                                        CloudType_::Ptr &calib_board);
        void regionGrowSeg(CloudType_::Ptr &cloud_in_, vector<pcl::PointIndices> &clusters_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_result_);
        CloudType_::Ptr IntensityFilter(CloudType_::Ptr& cloud_in, float rm_range_min, float rm_range_max);

        #ifdef STATIC_ANALYSE
        void visualize_regist(CloudType_::Ptr& source, CloudType_::Ptr& target, CloudType_::Ptr& registed);
        void visualize_regist(pcl::PointCloud<pcl::PointXYZI>::Ptr& target, pcl::PointCloud<pcl::PointXYZI>::Ptr& registed, double sigma1, double sigma2, size_t dot_size = 1, string viewer_title = "Registration Result");
        void visualize_regist(pcl::PointCloud<pcl::PointXYZI>::Ptr& target, pcl::PointCloud<pcl::PointXYZI>::Ptr& registed, size_t dot_size = 1, string viewer_title = "Registration Result");
        void visualEigenvalues(Eigen::Vector4f& pcaCentroid, Eigen::Matrix3f& eigenVectors_, pcl::visualization::PCLVisualizer& viewer, const int viewpoint = 0, string arrow_name = "arrow");
        void showPointXYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, size_t dot_size, string viewer_title);
        #endif
};


bool AutoDetectLaser::detectCalibBoard(CloudType_::Ptr &cloud_in, 
                                        CloudType_::Ptr &calib_board)
{
    CloudType_::Ptr plane_cloud(new CloudType_),    // To store all detected planes
                                        cloud_f(new CloudType_),        // Temp pc used for swaping
                                        cloud2(new CloudType_),         // Temp pc used for swaping
                                        // calib_board_boundary_(new CloudType_),
                                        detected_boards(new CloudType_);
    bool detectable = false;

    #ifdef STATIC_ANALYSE
    showPointXYZI(cloud_in, 2, "Raw Cloud_in");
    #endif
    
    // ************************ 1. x-filter ***********************
    CloudType_::Ptr x_filtered (new CloudType_);
    pcl::PassThrough<PointType_> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(remove_x_min_, remove_x_max_);
    pass_x.setInputCloud(cloud_in);
    pass_x.setNegative (true);
    pass_x.filter(*x_filtered);
    pcl::copyPointCloud(*x_filtered, *x_filtered_);

    // ********************** 2. Downsampling *********************
    if(use_vox_filter_)
    {   
        CloudType_::Ptr cloud_to_divide (new CloudType_);
        pcl::VoxelGrid<PointType_> sor;

        // The size of point cloud is too large. It's neccessary to divide it into multiple blocks to do downsampling
        PointType_ minP, maxP;
        pcl::getMinMax3D(*x_filtered, minP, maxP);
        double dis_x = maxP.x - minP.x,
                dis_y = maxP.y - minP.y,
                dis_z = maxP.z - minP.z;
        double volumn = dis_x * dis_y *dis_z;
        int block_num = ceill(volumn / 3000);
        if(DEBUG)
        {   
            cout << "x_min = " << minP.x << "\tx_max = " << maxP.x << endl;
            cout << "source cloud volumn = " << volumn << "\tblock_num = " << block_num << endl;
        }
        double add_dx = dis_x/block_num;

        pcl::copyPointCloud(*x_filtered, *cloud_to_divide);
        CloudType_::Ptr dx_filtered (new CloudType_);
        CloudType_::Ptr block_filtered (new CloudType_);
        
        for(auto dx_min = minP.x; dx_min < maxP.x ; dx_min += add_dx)
        {
            // divide into blocks
            auto dx_max = dx_min + add_dx;
            pcl::PassThrough<PointType_> pass_dx;
            pass_dx.setFilterFieldName("x");
            pass_dx.setFilterLimits(dx_min, dx_max);
            pass_dx.setInputCloud(cloud_to_divide);
            pass_dx.setNegative (false);    // inliers
            pass_dx.filter(*dx_filtered);
            
            pass_dx.setNegative(true);      // outliers
            pass_dx.filter(*cloud_to_divide);

            sor.setInputCloud(dx_filtered);
            sor.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
            sor.filter(*block_filtered);
            

            *cloud2 += *block_filtered;
        }
        if(DEBUG) 
            cout << "PointCloud size after filter: " << cloud2->points.size() << endl;
    }
    else
        pcl::copyPointCloud(*x_filtered, *cloud2);
    // if(auto_mode_) showPointXYZI(cloud2, 1, "pointcloud after voxel");
    pcl::copyPointCloud(*cloud2, *ds_filtered_);

    // ******************** 3. Gaussion filter *******************
    CloudType_::Ptr cloud_gauss_filtered (new CloudType_);
    if(use_gauss_filter_)
    {
        // ------ Implementation of convolution filtering based on Gaussian kernel function ------
        pcl::filters::GaussianKernel<PointType_, PointType_> kernel;
        kernel.setSigma(gauss_k_sigma_);    // The standard deviation of the Gaussian function, which determines the width of the function
        kernel.setThresholdRelativeToSigma(gauss_k_thre_rt_sigma_); //　Set the distance threshold relative to the sigma parameter
        kernel.setThreshold(gauss_k_thre_); //　Set the distance threshold, if the distance between points is greater than the threshold, these points will not be considered

        pcl::search::KdTree<PointType_>::Ptr gauss_tree(new pcl::search::KdTree<PointType_>);
        gauss_tree->setInputCloud(cloud2);
        
        // ------ Set Convolution parameters ------
        pcl::filters::Convolution3D<PointType_, PointType_, pcl::filters::GaussianKernel<PointType_, PointType_>> convolution;
        convolution.setKernel(kernel); // Set Convolution Kernel
        convolution.setInputCloud(cloud2);
        convolution.setNumberOfThreads(8);
        convolution.setSearchMethod(gauss_tree);
        convolution.setRadiusSearch(gauss_conv_radius_);

        convolution.convolve(*cloud_gauss_filtered);
        // if(auto_mode_) showPointXYZI(cloud_gauss_filtered, 1, "after gauss filter"); 

        pcl::copyPointCloud(*cloud_gauss_filtered, *cloud2);
        pcl::copyPointCloud(*cloud_gauss_filtered, *gs_filtered_);
    }

    // ************************ 4.Euclidean Cluster ******************************
    pcl::search::KdTree<PointType_>::Ptr tree(new pcl::search::KdTree<PointType_>);
    tree->setInputCloud(cloud2);
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType_> euclidean_cluster;
    euclidean_cluster.setClusterTolerance(cluster_tole_);   // Set the nearest neighbor search radius to 5cm
    euclidean_cluster.setMinClusterSize(cluster_size_min_);
    euclidean_cluster.setMaxClusterSize(cluster_size_max_);
    euclidean_cluster.setSearchMethod(tree);
    euclidean_cluster.setInputCloud(cloud2);
    euclidean_cluster.extract(cluster_indices);


    // ********************** 5. Plane Segmentation (in each cluster) ******************
    pcl::SACSegmentation<PointType_> plane_segmentation;
    plane_segmentation.setOptimizeCoefficients(true);   // Reestimate model parameters using interior points
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(Pseg_dis_thre_);
    plane_segmentation.setMaxIterations(Pseg_iter_num_);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::ExtractIndices<PointType_> extract_plane;
    int seg_num = 0, pos_num = 0, true_pos_num = 0;
    if(DEBUG2) cout << cluster_indices.size() << " clusters found from "  << cloud2->points.size() << " points in cloud" << endl;
    
    for(auto it = cluster_indices.begin(); it < cluster_indices.end(); it++)
    {
        CloudType_::Ptr cloud_cluster(new CloudType_);
        PointType_ tmp_p;
        for(auto pit = it->indices.begin(); pit < it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud2->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if(DEBUG) ROS_WARN("PointCloud represneting the Cluster: %d data points.", cloud_cluster->points.size());
        // if(auto_mode_) showPointXYZI(cloud_cluster, 1, "cloud cluster");

        // ------ voxel2 to uniform pcl ------
        if(use_vox_filter_)
        {
            CloudType_::Ptr voxel2_filtered(new CloudType_);
            pcl::VoxelGrid<PointType_> sor2;
            sor2.setInputCloud(cloud_cluster);
            sor2.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
            sor2.filter(*voxel2_filtered);
            pcl::copyPointCloud(*voxel2_filtered, *cloud_cluster);
        }

        // ******************* statistical filter ******************
        if(use_statistic_filter_)
        {
            CloudType_::Ptr cloud_s_filtered(new CloudType_);
            pcl::StatisticalOutlierRemoval<PointType_> sor;
            sor.setInputCloud(cloud_cluster);
            sor.setMeanK(sor_MeanK_);
            sor.setStddevMulThresh(sor_StddevMulThresh_);
            sor.filter(*cloud_s_filtered);
            if(DEBUG) cout << "cluster size after statistic filter = " << cloud_s_filtered->points.size() << endl;
            pcl::copyPointCloud(*cloud_s_filtered, *cloud_cluster);
            // if(auto_mode_) visualPointXYZI(cloud_cluster, 1, "cloud cluster after statistic filter");	
        }

        int full_cloud_size = cloud_cluster->points.size();
        while(cloud_cluster->points.size() > 0 && cloud_cluster->points.size() > (Pseg_size_min_ * full_cloud_size))
        {
            bool is_detected_calib = false;
            pcl::console::TicToc t_seg;
            double Pseg_time_ = 0.0;
            t_seg.tic();
            plane_segmentation.setInputCloud (cloud_cluster);
            plane_segmentation.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                ROS_WARN("<<<<<< [Laser] Could not estimate a planar model for the given dataset.");
                break;
            }
            seg_num++;
            Pseg_time_ = t_seg.toc();
            // Pseg_time_v.push_back(Pseg_time_);
            if(DEBUG2) 
            {
                ROS_WARN("Segmentation No.%d\tspend[%fms]", seg_num, Pseg_time_);
                // cout << "segmentation No." << seg_num << "\tspend [" << t_seg.toc() << "ms]" <<endl;
                cout << "number of point clouds in the plane: " << inliers->indices.size() << endl;
            }

            extract_plane.setInputCloud(cloud_cluster);
            extract_plane.setIndices (inliers);
            extract_plane.setNegative (false);    //extract_plane inliers
            extract_plane.filter (*plane_cloud);

            #ifdef STATIC_ANALYSE
            showPointXYZI(plane_cloud, 2, "Unknown Plane");
            #endif 
            // ************************* 5.1 intensity filter ***************************
            CloudType_::Ptr filtered_plane (new CloudType_);
            if(use_i_filter_)
            {
                filtered_plane = IntensityFilter(plane_cloud, i_filter_out_min_, i_filter_out_max_);
                #ifdef STATIC_ANALYSE
                showPointXYZI(filtered_plane, 2, "Intensity Filter Result");
                #endif 
            }
            else
                pcl::copyPointCloud(*plane_cloud, *filtered_plane);

            // ************************* 5.2 coloring for visiualization ***************************
            pcl::PointCloud<pcl::PointXYZI>::Ptr colored_i_plane(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*plane_cloud, *colored_i_plane);
            for(auto it = colored_i_plane->points.begin(); it < colored_i_plane->points.end(); it++)
            {
                it->intensity = seg_num;
            }
            *colored_i_planes_ += *colored_i_plane;

            // ************************* 6. justifying if it's the calib board ****************
            CloudType_::Ptr boundary_registed(new CloudType_);
            CloudType_::Ptr boundary(new CloudType_);
            double check_time_ = 0.0;
            pcl::console::TicToc t_check;
            t_check.tic();
            is_detected_calib = isCalibBoard(filtered_plane, boundary, boundary_registed);
            check_time_ = t_check.toc();
            // check_time_v.push_back(check_time_);
            if(is_detected_calib)
            {
                // ROS_WARN("<<<<<<<<<<<<< [LASER] Have found the calib borad point cloud!!!");
                detectable = true;
                pos_num ++;
                Tr_calib2tpl_ = Tr_ukn2tpl_;
                *detected_boards += *filtered_plane;
                // if(DEBUG) showPointXYZI(detected_boards,2, "positive boards");
                pcl::copyPointCloud(*filtered_plane, *calib_board);
                pcl::copyPointCloud(*boundary_registed, *calib_board_boundary_registed_);
                pcl::copyPointCloud(*boundary, *calib_board_boundary_);
            }

            extract_plane.setNegative (true);     // extract_plane outliers
            extract_plane.filter(*cloud_f);
            cloud_cluster.swap(cloud_f);
            if(DEBUG) ROS_INFO("Remianing %d points in cloud", cloud_cluster->points.size());
        }
    }    
    #ifdef STATIC_ANALYSE
    showPointXYZI(colored_i_planes_, 2, "Plane Segmentation Result");
    #endif
    // if(!detectable)
    //     ROS_WARN("<<<<<<<<<<<<< [LASER] CANNOT find the calib borad!");
    pcl::copyPointCloud(*detected_boards, *calib_board);
    return detectable;
}


bool AutoDetectLaser::detectCalibBoardRG(CloudType_::Ptr &cloud_in, 
                                        CloudType_::Ptr &calib_board)
{
    CloudType_::Ptr plane_cloud(new CloudType_),    // To store all detected planes
                                        cloud_f(new CloudType_),        // Temp pc used for swaping
                                        cloud2(new CloudType_),         // Temp pc used for swaping
                                        // calib_board_boundary_(new CloudType_),
                                        detected_boards(new CloudType_);
    bool detectable = false;
    // ************************ 1. x-filter ***********************
    CloudType_::Ptr x_filtered (new CloudType_);
    pcl::PassThrough<PointType_> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(remove_x_min_, remove_x_max_);
    pass_x.setInputCloud(cloud_in);
    pass_x.setNegative (true);
    pass_x.filter(*x_filtered);
    pcl::copyPointCloud(*x_filtered, *x_filtered_);

    // ********************** 2. Downsampling *********************
    if(use_vox_filter_)
    {   
        CloudType_::Ptr cloud_to_divide (new CloudType_);
        pcl::VoxelGrid<PointType_> sor;

        // 输入点云空间体量过大，需分块后进行下采样
        PointType_ minP, maxP;
        pcl::getMinMax3D(*x_filtered, minP, maxP);
        double dis_x = maxP.x - minP.x,
                dis_y = maxP.y - minP.y,
                dis_z = maxP.z - minP.z;
        double volumn = dis_x * dis_y *dis_z;
        int block_num = ceill(volumn / 3000);
        if(DEBUG)
        {   
            cout << "x_min = " << minP.x << "\tx_max = " << maxP.x << endl;
            cout << "source cloud volumn = " << volumn << "\tblock_num = " << block_num << endl;
        }
        double add_dx = dis_x/block_num;

        pcl::copyPointCloud(*x_filtered, *cloud_to_divide);
        CloudType_::Ptr dx_filtered (new CloudType_);
        CloudType_::Ptr block_filtered (new CloudType_);
        
        for(auto dx_min = minP.x; dx_min < maxP.x ; dx_min += add_dx)
        {
            // divide into blocks
            auto dx_max = dx_min + add_dx;
            pcl::PassThrough<PointType_> pass_dx;
            pass_dx.setFilterFieldName("x");
            pass_dx.setFilterLimits(dx_min, dx_max);
            pass_dx.setInputCloud(cloud_to_divide);
            pass_dx.setNegative (false);    // inliers
            pass_dx.filter(*dx_filtered);
            
            pass_dx.setNegative(true);      // outliers
            pass_dx.filter(*cloud_to_divide);

            sor.setInputCloud(dx_filtered);
            sor.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
            sor.filter(*block_filtered);
            

            *cloud2 += *block_filtered;
        }
        if(DEBUG) 
            cout << "PointCloud size after filter: " << cloud2->points.size() << endl;
    }
    else
        pcl::copyPointCloud(*x_filtered, *cloud2);
    // if(auto_mode_) showPointXYZI(cloud2, 1, "pointcloud after voxel");

    pcl::copyPointCloud(*cloud2, *ds_filtered_);
    // ******************** 3. Gaussion filter *******************
    CloudType_::Ptr cloud_gauss_filtered (new CloudType_);
    if(use_gauss_filter_)
    {
        // **************** 基于高斯核函数的卷积滤波实现 *****************
        pcl::filters::GaussianKernel<PointType_, PointType_> kernel;
        kernel.setSigma(gauss_k_sigma_);    // 高斯函数的标准方差，决定函数的宽度
        kernel.setThresholdRelativeToSigma(gauss_k_thre_rt_sigma_); //　设置相对sigma参数的距离阈值
        kernel.setThreshold(gauss_k_thre_); //　设置距离阈值，若点间距离大于阈值则不予考虑
        // cout << "Kernel made" << endl;

        pcl::search::KdTree<PointType_>::Ptr gauss_tree(new pcl::search::KdTree<PointType_>);
        gauss_tree->setInputCloud(cloud2);
        // cout << "KdTree made" << endl;
        
        // *************** 设置Convolution相关参数 *****************
        pcl::filters::Convolution3D<PointType_, PointType_, pcl::filters::GaussianKernel<PointType_, PointType_>> convolution;
        convolution.setKernel(kernel); // 设置卷积核
        convolution.setInputCloud(cloud2);
        convolution.setNumberOfThreads(8);
        convolution.setSearchMethod(gauss_tree);
        convolution.setRadiusSearch(gauss_conv_radius_);
        // cout << "Convolution start" << endl;

        convolution.convolve(*cloud_gauss_filtered);
        // cout << "cluster size after gauss filter: " << cloud_gauss_filtered->points.size() << endl;
        // if(auto_mode_) showPointXYZI(cloud_gauss_filtered, 1, "after gauss filter"); 

        pcl::copyPointCloud(*cloud_gauss_filtered, *cloud2);

        pcl::copyPointCloud(*cloud_gauss_filtered, *gs_filtered_);
    }

    // ************************ 4. RG plane segmentation ******************************
    vector<pcl::PointIndices> cluster_indices;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_planes_ (new pcl::PointCloud<pcl::PointXYZRGB>);

    regionGrowSeg(cloud2, cluster_indices, colored_planes_);

    pcl::ExtractIndices<PointType_> extract_plane;
 

    int seg_num = 0, pos_num = 0, true_pos_num = 0;
    if(DEBUG2) cout << cluster_indices.size() << " clusters found from "  << cloud2->points.size() << " points in cloud" << endl;
    
    for(auto it = cluster_indices.begin(); it < cluster_indices.end(); it++)
    {
        CloudType_::Ptr cloud_cluster(new CloudType_);
        // for(auto pit = it->indices.begin(); pit < it->indices.end(); pit++)
        // {
        //     cloud_cluster->points.push_back(cloud2->points[*pit]);
        // }
        // cloud_cluster->width = cloud_cluster->points.size();
        // cloud_cluster->height = 1;
        // cloud_cluster->is_dense = true;
        pcl::PointIndices::Ptr cluster_indice_ptr(new pcl::PointIndices(*it));
        extract_plane.setInputCloud(cloud2);
        extract_plane.setIndices (cluster_indice_ptr);
        extract_plane.setNegative (false);    //extract_plane inliers
        extract_plane.filter (*plane_cloud);

        if(DEBUG) ROS_WARN("PointCloud represneting the Cluster: %d data points.", plane_cloud->points.size());
        // if(auto_mode_) showPointXYZI(cloud_cluster, 1, "cloud cluster");

        // ******************* voxel2 to uniform pcl ******************
        if(use_vox_filter_)
        {
            CloudType_::Ptr voxel2_filtered(new CloudType_);
            pcl::VoxelGrid<PointType_> sor2;
            sor2.setInputCloud(plane_cloud);
            sor2.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
            sor2.filter(*voxel2_filtered);
            pcl::copyPointCloud(*voxel2_filtered, *plane_cloud);
        }

         // ******************* statistical filter ******************
        if(use_statistic_filter_)
        {
            CloudType_::Ptr cloud_s_filtered(new CloudType_);
            pcl::StatisticalOutlierRemoval<PointType_> sor;
            sor.setInputCloud(plane_cloud);
            sor.setMeanK(sor_MeanK_);
            sor.setStddevMulThresh(sor_StddevMulThresh_);
            sor.filter(*cloud_s_filtered);
            if(DEBUG) cout << "cluster size after statistic filter = " << cloud_s_filtered->points.size() << endl;
            pcl::copyPointCloud(*cloud_s_filtered, *plane_cloud);
            // if(auto_mode_) visualPointXYZI(cloud_cluster, 1, "cloud cluster after statistic filter");	
        }

        bool is_detected_calib = false;    // 算法检测是否是标定板
        // Pseg_time_v.push_back(Pseg_time_);



        // ************************* 5.1 intensity filter ***************************
        CloudType_::Ptr filtered_plane(new CloudType_);
        if(use_i_filter_)
            filtered_plane = IntensityFilter(plane_cloud, i_filter_out_min_, i_filter_out_max_);
        else
            pcl::copyPointCloud(*plane_cloud, *filtered_plane);

        // ************************* 5.2 coloring for visiualization ***************************


        // ************************* 6. justifying if it's the calib board ****************
        CloudType_::Ptr boundary_registed(new CloudType_);
        CloudType_::Ptr boundary(new CloudType_);
        double check_time_ = 0.0;
        pcl::console::TicToc t_check;
        t_check.tic();
        // is_detected_calib = isCalibBoard(filtered_plane, calib_template_, boundary_registed);
        is_detected_calib = isCalibBoard(filtered_plane, boundary, boundary_registed);
        check_time_ = t_check.toc();
        // check_time_v.push_back(check_time_);
        if(is_detected_calib)
        {
            // ROS_WARN("<<<<<<<<<<<<< [LASER] Have found the calib borad point cloud!!!");
            detectable = true;
            pos_num ++;
            Tr_calib2tpl_ = Tr_ukn2tpl_;
            *detected_boards += *filtered_plane;
            // if(DEBUG) showPointXYZI(detected_boards,2, "positive boards");
            pcl::copyPointCloud(*filtered_plane, *calib_board);
            pcl::copyPointCloud(*boundary_registed, *calib_board_boundary_registed_);
            pcl::copyPointCloud(*boundary, *calib_board_boundary_);
        }

        // if(DEBUG) ROS_INFO("Remianing %d points in cloud", cloud_cluster->points.size());
    }
    // if(!detectable)
    //     ROS_WARN("<<<<<<<<<<<<< [LASER] CANNOT find the calib borad!");
    pcl::copyPointCloud(*detected_boards, *calib_board);
    return detectable;
}


void AutoDetectLaser::regionGrowSeg(CloudType_::Ptr &cloud_in_, vector<pcl::PointIndices> &clusters_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_result_)
{
    pcl::NormalEstimation<PointType_, pcl::Normal> normEst;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointType_>::Ptr tree(new pcl::search::KdTree<PointType_>);
    normEst.setSearchMethod(tree);
    normEst.setInputCloud(cloud_in_);
    normEst.setKSearch(this->reforn_);  // number of points to search
    // normEst.setRadiusSearch(reforn_);
    normEst.compute(*normals);

    pcl::RegionGrowing<PointType_, pcl::Normal> reg;
    reg.setMinClusterSize(cluster_size_min_);
    reg.setMaxClusterSize(cluster_size_max_);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(RG_neighbor_n_);
    reg.setInputCloud(cloud_in_);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(pcl::deg2rad(RG_smooth_thre_deg_));
    reg.setCurvatureThreshold(RG_curve_thre_);

    reg.extract(clusters_);
    colored_result_ = reg.getColoredCloud();
}


bool AutoDetectLaser::isCalibBoard(CloudType_::Ptr& cloud, 
                CloudType_::Ptr& cloud_boundary,
                CloudType_::Ptr& cloud_boundary_registed)
{
    if(cloud->points.size() == 0)
    {
        if(DEBUG) cerr<< "[isCalibBoard] cloud_in is empty!" << endl;
        return false;
    }
    rmse_ukn2tpl_ = rmse_tpl2ukn_ = icp_score_ = -1.0;
    // ******************* Voxel Filter **********************
    // if(use_voxel_filter)
    // {
    //     CloudType_::Ptr cloud_filtered (new CloudType_);
    //     pcl::VoxelGrid<PointType_> sor;
    //     sor.setInputCloud(cloud);
    //     sor.setLeafSize(0.01f, 0.01f, 0.01f);
    //     sor.filter(*cloud_filtered);
    //     if(DEBUG) cout << "PointCloud size after filter: " << cloud_filtered->points.size() << endl;
    //     pcl::copyPointCloud(*cloud_filtered, *cloud);
    // }
        

    //********************compute the PCA transform matrix*************
    pcl::console::TicToc time;
    time.tic();
    Eigen::Matrix4f PCA_Transform = Eigen::Matrix4f::Identity();
    // PCA_Transform = PCARegistration(cloud_to_test, calib_template_);
    PCA_Transform = PCARegistration(cloud, calib_template_);
    if(DEBUG2) cout << "the PCA computation spend [ " << time.toc() << "ms ]" << endl;
    if(DEBUG) cout << "transform matrix = \n" << PCA_Transform << endl;

    CloudType_::Ptr PCARegisted(new CloudType_);
    // pcl::transformPointCloud(*cloud_to_test, *PCARegisted, PCA_Transform);
    pcl::transformPointCloud(*cloud, *PCARegisted, PCA_Transform);
    // for(auto p:PCARegisted->points)
    // {
    //     p.z = 0.0;
    // }
    pcl::copyPointCloud(*PCARegisted, *pca_regist_pc_);


    //****************** extract boundary **************
    CloudType_::Ptr PCARegisted_boundary(new CloudType_);
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries (new pcl::PointCloud<pcl::Boundary>);   //储存边界估计结果
    pcl::console::TicToc tt;
    tt.tic();

    this->estimateBorders(PCARegisted, boundaries);
    if(DEBUG2) std::cout << "estimateBorders spend [ " << tt.toc() << " ms ]" << std::endl;
    for(auto p = boundaries->begin(); p < boundaries->end(); p++)
    {
        if(p->boundary_point > 0)   PCARegisted_boundary->push_back(PCARegisted->points[p-boundaries->begin()]);
    }
    if(DEBUG) cout << "size of boudary: " << PCARegisted_boundary->points.size() << endl;
    if(PCARegisted_boundary->points.size() <= 3)
    {
        if(DEBUG) ROS_WARN("This plane is invalid");
        return false;
    }
    // pcl::transformPointCloud(*PCARegisted_boundary, *cloud_boundary_registed, PCA_Transform.inverse());
    pcl::copyPointCloud(*PCARegisted_boundary, *pca_regist_boundary_);
    pcl::transformPointCloud(*PCARegisted_boundary, *cloud_boundary, PCA_Transform.inverse());

    //****************** ICP *****************
    pcl::console::TicToc time2;
    time2.tic();
    pcl::IterativeClosestPoint<PointType_, PointType_> icp;
    icp.setInputSource(PCARegisted_boundary);
    icp.setInputTarget(calib_template_);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(1);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setMaximumIterations(100);
    icp.setUseReciprocalCorrespondences(true);
    CloudType_::Ptr icp_cloud(new CloudType_);
    icp.align(*icp_cloud);
    if (icp.hasConverged()) 
    {
        if(DEBUG2) ROS_INFO("ICP has converged!");
        icp_score_ = icp.getFitnessScore();
        if(DEBUG2) cout << "\nICP has converged, score is " << icp_score_ << endl;
        
        Tr_ukn2tpl_ = icp.getFinalTransformation() * PCA_Transform;
        
    }
    else 
    {
        if(DEBUG2)ROS_WARN("ICP hasn't converged!");
        icp_score_ = -1.0;
    }
    if(DEBUG2) cout << "Applied " << 100 << " ICP iterations in [ " << time2.toc() << " ms ]" << endl;
    if(DEBUG)   
    {
        cout << "ICP Transformation: \n" << icp.getFinalTransformation() << endl;
        cout << "The final TR =\n" << Tr_ukn2tpl_ << endl;
    }       
    pcl::copyPointCloud(*icp_cloud, *cloud_boundary_registed);
    pcl::copyPointCloud(*icp_cloud, *icp_regist_boundary_);
    

    // ************************* difference assesment *************************
    if(DEBUG)
    {
        cout << "size of icp_cloud: " << icp_cloud->points.size() << endl;
        cout << "size of template: " << calib_template_->points.size() << endl; 
    }
    float rmse_ukn2tpl, rmse_tpl2ukn, rmse_mean;
    rmse_ukn2tpl = ComputeDifference(icp_cloud, calib_template_);
    rmse_tpl2ukn = ComputeDifference(calib_template_, icp_cloud);
    rmse_ukn2tpl_ = rmse_ukn2tpl;
    rmse_tpl2ukn_ = rmse_tpl2ukn;
    rmse_mean = (rmse_ukn2tpl + rmse_tpl2ukn) / (float)2;
    if(DEBUG3)
    {
        ROS_INFO("The rmse of unknown cloud to template cloud is: %f", rmse_ukn2tpl);
        ROS_INFO("The rmse of template cloud to unknown cloud is: %f", rmse_tpl2ukn);
        ROS_INFO("The mean of rmse is: %f", rmse_mean);
    }

    #ifdef STATIC_ANALYSE
    // visualize_regist(cloud_boundary_registed, calib_template_, icp_cloud);
    // showPointXYZI(cloud, 2, "Unknown Plane");
    visualize_regist(calib_template_, PCARegisted_boundary, 2, "PCA Registration Result");
    visualize_regist(calib_template_, icp_cloud, rmse_ukn2tpl, rmse_tpl2ukn, 2, "ICP Registration Result");
    #endif

    if(rmse_ukn2tpl <= rmse_ukn2tpl_thre_ && rmse_tpl2ukn <= rmse_tpl2ukn_thre_)
        return true;
    else
        return false;

     
    // visualize_regist(cloud_boundary_registed, calib_template_, icp_cloud);

}






Eigen::Matrix4f AutoDetectLaser::PCARegistration(CloudType_::Ptr& source_cloud, CloudType_::Ptr& target_cloud)
{
    // Eigen::Vector4f C_source, C_target;  // Centroid
    // Eigen::Matrix3f U_source, U_target;  // Eiegnvectors
    // Eigen::Vector3f lamda_source, lamda_target;    // Eiegnvalues

    ComputeEigenVectorPCA(source_cloud, C_source, U_source, lamda_source);
    ComputeEigenVectorPCA(target_cloud, C_target, U_target, lamda_target);

    if(DEBUG)
    {   
        ROS_INFO("PCA computing finished!");

        cout << "-------The Centroid-------" << endl;
        cout << "source cloud:\t" << C_source << endl;
        cout << "target cloud:\t" << C_target << endl;

        cout << "-------Eignevectors-------" << endl;
        cout << "source cloud:\n" << U_source << endl;
        cout << "target cloud:\n" << U_target << endl;

        cout << "-------Eigenvalues-------" << endl;
        cout << "source cloud:\t" << lamda_source.transpose() << endl;
        cout << "target cloud:\t" << lamda_target.transpose() << endl;
    }

    Eigen::Matrix4f final_TR = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R0 = U_target * U_source.inverse();
    Eigen::Vector3f T0 = C_target.head<3>() - R0 * (C_source.head<3>());
    Isometry3f TR0 = Isometry3f::Identity();
    TR0.rotate(R0);
    TR0.pretranslate(T0);
    
    final_TR = TR0.matrix();
    return final_TR;

}


float AutoDetectLaser::CalculateRMSE(std::vector<float> data)
{
    int N = data.size();
    float rmse = 0, mean = 0, sum = 0;
    for(auto p:data)
    {
        sum += p;
    }
    mean = sum / (float)N;
    if(DEBUG) cout << "the mean is " << mean << endl;

    sum = 0;
    for(auto p:data)
    {
        sum += pow(p - mean, 2);
    }
    rmse = sqrt((sum / (float)N));

    return rmse;
}


float AutoDetectLaser::ComputeDifference(CloudType_::Ptr& source, CloudType_::Ptr& target)
{
    std::vector<float> distance;
    pcl::KdTreeFLANN<PointType_> kdtree;
    kdtree.setInputCloud(target);
    for(auto p = source->begin(); p < source->end(); p++)
    {
        PointType_ searchPoint = *p;
        std::vector<int> pointIdxKNNSearch(1);          // 保存近邻点的索引
        std::vector<float> pointKNNSquareDistance(1);   // 保存每个近邻点与查找点之间的欧式距离平方

        if(kdtree.nearestKSearch(searchPoint, 1, pointIdxKNNSearch, pointKNNSquareDistance))
        {
            // if(DEBUG)   cout << p - source->begin() << ". " << "for point: " << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << endl;
            for(auto i = pointIdxKNNSearch.begin(); i < pointIdxKNNSearch.end(); i++)
            {
                float dist = sqrt(pointKNNSquareDistance[i-pointIdxKNNSearch.begin()]);
                // if(DEBUG)
                // {
                //     cout << "\t" << (*target)[*i].x << " " << (*target)[*i].y << " " << (*target)[*i].z
                //     << "\tdistance: " << dist << endl; 
                // }
                distance.push_back(dist);
            }   
        }
    }
    if(DEBUG) cout << "distance vector size = " << distance.size() << endl;
    float rmse = CalculateRMSE(distance);
    
    return rmse;
}




void AutoDetectLaser::RemoveFloor(CloudType_::Ptr& cloud_in, CloudType_::Ptr& cloud_out, float part)
{
    PointType_ min;
    PointType_ max;
    pcl::getMinMax3D(*cloud_in, min, max);

    if(DEBUG)   cout << "min_z = " << min.z << "\t max_z = " << max.z << endl;

    double passthrough_z_min = min.z + (double)part * (max.z - min.z);
    double passthrough_z_max = max.z + (double)part * (max.z - min.z);
    pcl::PassThrough<PointType_> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(passthrough_z_min, passthrough_z_max);
    pass.setInputCloud(cloud_in);
    pass.setNegative (false);
    pass.filter(*cloud_out);

    return;
}


CloudType_::Ptr AutoDetectLaser::IntensityFilter(CloudType_::Ptr& cloud_in, float rm_range_min, float rm_range_max)
{
    // vector<float> v_intensity;
    // for(auto p:cloud_in->points)
    // {
    //     v_intensity.push_back(p.intensity);
    // }
    // sort(v_intensity.begin(), v_intensity.end());   // 从小到大
    // float max_i = *(v_intensity.end()-1), min_i = *v_intensity.begin();
    // if(DEBUG)
    // {
    //     cout << "the size of v_intensity: " << v_intensity.size() << endl;
    //     cout << "the max intensity: " << max_i << "\t the min intensity: " << min_i << endl;
    // } 

    CloudType_::Ptr filtered_cloud (new CloudType_);
    pcl::PassThrough<PointType_> pass;
    // float min_to_rm = min_i + remove_part * (max_i - min_i), max_to_rm = max_i;
    float min_to_rm = rm_range_min, max_to_rm = rm_range_max;
    
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(min_to_rm, max_to_rm);
    pass.setNegative(true);
    pass.setInputCloud(cloud_in);
    pass.filter(*filtered_cloud);
    if(DEBUG)
    {
        cout << "min_to_rm: " << min_to_rm << "\t" << "max_to_rm: " << max_to_rm << endl;
        cout << "size after intensity filter: " << filtered_cloud->size() << endl;
    }
    
    
    return filtered_cloud;
}





#ifdef STATIC_ANALYSE
void AutoDetectLaser::visualize_regist(CloudType_::Ptr& source, CloudType_::Ptr& target, CloudType_::Ptr& registed)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
    int v1 = 0, v2 = 1;
    viewer->setWindowName("Registration Result");
    viewer->createViewPort(0, 0, 0.5, 1,v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("Raw point cloud", 10, 10, "v1_text", v1);
    viewer->addText("Registed point cloud", 10, 10, "v2_text", v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointType_> src_h(source, 255, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType_> tgt_h(target, 0, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType_> trans_h(registed, 255, 255, 0);

    viewer->addPointCloud(source, src_h, "source cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
    viewer->addPointCloud(registed, trans_h, "registed cloud", v2);
    visualEigenvalues(C_source, U_source, *viewer, v1, "arrow_source");
    visualEigenvalues(C_target, U_target, *viewer, v1);
    visualEigenvalues(C_target, U_target, *viewer, v2);


    viewer->addCoordinateSystem(0.1);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    viewer->close();

    return;
}

void AutoDetectLaser::visualize_regist(pcl::PointCloud<pcl::PointXYZI>::Ptr& target, pcl::PointCloud<pcl::PointXYZI>::Ptr& registed, double sigma1, double sigma2, size_t dot_size, string viewer_title)
{
    cout << "[visualize_regist]" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
    viewer->setWindowName(viewer_title);
    viewer->setBackgroundColor(1, 1, 1);
    // viewer->setFullScreen(true);
    viewer->setSize(640, 480);
    viewer->setCameraPosition(0, 0, 6, 0, 0, 0, 0, 1, 0);
    
    viewer->addText("sigma1 = " + to_string(sigma1), 5, 27, 25, 0, 0, 1, "sigma1_text");
    viewer->addText("sigma2 = " + to_string(sigma2), 5, 2, 25, 1, 0, 0, "sigma2_text");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tgt_h(target, 0, 255, 255);pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> trans_h(registed, 255, 0, 255);

    viewer->addPointCloud(target, tgt_h, "target cloud");
    viewer->addPointCloud(registed, trans_h, "registed cloud");
    visualEigenvalues(C_target, U_target, *viewer);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dot_size, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dot_size, "registed cloud");

    viewer->addCoordinateSystem(0.1);
    while(!viewer->wasStopped())
    {
        // cout << "[visualize_regist] while" << endl;

        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    viewer->close();
    cout << "[visualize_regist] close" << endl;

    return;
}

void AutoDetectLaser::visualize_regist(pcl::PointCloud<pcl::PointXYZI>::Ptr& target, pcl::PointCloud<pcl::PointXYZI>::Ptr& registed, size_t dot_size, string viewer_title)
{
    cout << "[visualize_regist]" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
    viewer->setWindowName(viewer_title);
    viewer->setBackgroundColor(1, 1, 1);
    // viewer->setFullScreen(true);
    viewer->setCameraPosition(0, 0, 6, 0, 0, 0, 0, 1, 0);
    viewer->setSize(640, 480);
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tgt_h(target, 0, 255, 255);pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> trans_h(registed, 255, 0, 255);

    viewer->addPointCloud(target, tgt_h, "target cloud");
    viewer->addPointCloud(registed, trans_h, "registed cloud");
    visualEigenvalues(C_target, U_target, *viewer);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dot_size, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dot_size, "registed cloud");

    viewer->addCoordinateSystem(0.1);
    while(!viewer->wasStopped())
    {
        // cout << "[visualize_regist] while" << endl;

        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    viewer->close();
    cout << "[visualize_regist] close" << endl;

    return;
}

void AutoDetectLaser::visualEigenvalues(Eigen::Vector4f& pcaCentroid, Eigen::Matrix3f& eigenVectors_, pcl::visualization::PCLVisualizer& viewer, const int viewpoint, string arrow_name)
{
    Eigen::Vector3f v1, v2, v3;
    v1 = eigenVectors_.col(0);
    v2 = eigenVectors_.col(1);
    v3 = eigenVectors_.col(2);

    // cout << v1.transpose() << endl;
    // cout << v2.transpose() << endl;
    // cout << v3.transpose() << endl;

    pcl::PointXYZ O, A, B, C;
    O.x = pcaCentroid[0];
    O.y = pcaCentroid[1];
    O.z = pcaCentroid[2];

    A.x = v1[0] + O.x;
    A.y = v1[1] + O.y;
    A.z = v1[2] + O.z;

    B.x = v2[0] + O.x;
    B.y = v2[1] + O.y;
    B.z = v2[2] + O.z;

    C.x = v3[0] + O.x;
    C.y = v3[1] + O.y;
    C.z = v3[2] + O.z;

    ostringstream name1;
    name1 << arrow_name << viewpoint * 3 + 1;
    viewer.addArrow<pcl::PointXYZ>(A, O, 1, 0, 0, false, name1.str(), viewpoint);
    ostringstream name2;
    name2 << arrow_name << viewpoint * 3 + 2; 
    viewer.addArrow<pcl::PointXYZ>(B, O, 0, 1, 0, false, name2.str(), viewpoint);
    ostringstream name3;
    name3 << arrow_name << viewpoint * 3 + 3; 
    viewer.addArrow<pcl::PointXYZ>(C, O, 0, 0, 1, false, name3.str(), viewpoint);

    return;
}

void AutoDetectLaser::showPointXYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, size_t dot_size, string viewer_title)
{
    cout << "[showPointXYZI]" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(viewer_title));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addText(viewer_title, 10, 10, "text");
    viewer->setSize(640, 480);
    viewer->setCameraPosition(-10, 0, 0, 3, 0, 0, 0, 0, 1);
    // viewer->addCoordinateSystem(0.1);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> x_color(cloud, "x");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> i_color(cloud, "intensity");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud,getRandomNumber(),getRandomNumber(),getRandomNumber());
    // viewer->addPointCloud<pcl::PointXYZI>(cloud, x_color, "board cloud");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, i_color, "board cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dot_size, "board cloud");
    
    while(!viewer->wasStopped())
    {
        // cout << "[showPointXYZI] while" << endl;
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    viewer->close();
    cout << "[showPointXYZI] close" << endl;

    return;
}
#endif

int getRandomNumber()
{
  int RandomNumber;
  RandomNumber = rand() % (256) + 0;
  return RandomNumber;
}

template <typename PointT>
void publishPC(const ros::Publisher& pc_pub_, const std_msgs::Header& header_, typename pcl::PointCloud<PointT>::Ptr& cloud_)
{
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_, output_msg);
    output_msg.header = header_;
    pc_pub_.publish(output_msg);
}

#endif