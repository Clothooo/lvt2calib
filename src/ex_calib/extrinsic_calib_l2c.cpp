#include <string>
#include <vector>
#include <algorithm>
#include <time.h>
#include <list>
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <lvt2calib/lvt2Calib.h>
#include <lvt2calib/slamBase.h>

#define DEBUG 0

using namespace std;

string camera_info_dir_ = "", calib_result_dir_ = "", features_info_dir_ = "", calib_result_name_ = "";
string ns_l, ns_c, ref_ns;
cv::Mat cameraMatrix_gazebo = (Mat_<double>(3,3) <<
          1624.7336487407558, 0.0, 640.5, 
          0.0, 1624.7336487407558, 480.5, 
          0.0, 0.0, 1.0
          );
cv::Mat cameraMatrix = Mat_<double>(3,3);

bool save_calib_file = false, is_multi_exp = false;
bool is_auto_mode = false;
vector<pcl::PointXYZ> lv_3d_for_reproj;
std::vector<cv::Point2f> lv_2d_projected, lv_2d_projected_min2d, lv_2d_projected_min3d,
                        cam_2d_for_reproj;
int sample_size = 0, sample_num = 0, sample_size_min = 0, sample_size_max = 0;
ostringstream os_calibfile_log, os_extrinsic_min3d, os_extrinsic_min2d;

list<int> sample_sequence;

lvt2Calib mycalib(L2C_CALIB);


// double calculateRADerr(double alpha_, double alpha_gt_)
// {
//     double delta = alpha_ - alpha_gt_;
//     double delta_ = (delta > M_PI)?(delta - M_PI * 2.0):((delta < -M_PI)?(delta + M_PI * 2.0):delta);

//     return delta_;
// }

// std::vector<double> calculateTransErr(std::vector<double> ground_truth_, std::vector<double> detected_)
// {
//     double err_tx_ = 0, err_ty_ = 0, err_tz_ = 0, err_total = 0;

//     err_tx_ = sqrt(pow(detected_[0]-ground_truth_[0], 2));
//     err_ty_ = sqrt(pow(detected_[1]-ground_truth_[1], 2));
//     err_tz_ = sqrt(pow(detected_[2]-ground_truth_[2], 2));
//     err_total = sqrt(pow(err_tx_, 2) + pow(err_ty_, 2) + pow(err_tz_, 2));

//     std::vector<double> err = {err_tx_, err_ty_, err_tz_, err_total};
//     return err;
// }

// double calculateAngularErr(Eigen::Matrix3f ground_truth_, Eigen::Matrix3f detected_)
// {
//     Eigen::Matrix3d m = (detected_.transpose() * ground_truth_).cast<double>();
//     double TR = m.trace();
//     double temp = (TR - 1) / 2.0;
//     double err = acos((temp < -1)?-1:((temp > 1)?1:temp));
//     return err;
// }

void ExtCalib(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud, std::vector<cv::Point2f> cam_2d_sorted)
{
    ROS_WARN("********** 2.0 calibration start **********");
    cout << "<<<<< 2.1 calibration via min3D " << endl;
    // min3D Calibration
    Eigen:: Matrix4d Tr_s2l_centroid_min_3d = mycalib.ExtCalib3D(laser_cloud, camera_cloud);
    Eigen::Matrix4d Tr_l2s_centroid = Tr_s2l_centroid_min_3d.inverse();
    // Get final transform from velo to camera (using centroid to do calibration)
    Eigen::Matrix4d Tr_s2c_centroid, Tr_l2c_centroid_min3d;
    Tr_s2c_centroid <<   0, -1, 0, 0,
    0, 0, -1, 0,
    1, 0, 0, 0,
    0, 0, 0, 1;
    // The final transformation matrix from lidar to camera frame (stereo_camera).
    Tr_l2c_centroid_min3d = Tr_s2c_centroid * Tr_l2s_centroid;
    cout << "Tr_laser_to_cam_centroid_min_3d = " << "\n" << Tr_l2c_centroid_min3d << endl;

    std::vector<double> calib_result_6dof_min3d = eigenMatrix2SixDOF(Tr_l2c_centroid_min3d);

    cout << "x, y, z, roll, pitch, yaw = " << endl;
    for(std::vector<double>::iterator it=calib_result_6dof_min3d.begin(); it<calib_result_6dof_min3d.end(); it++){
        cout  << (*it) << endl;
    }
    tf::Matrix3x3 tf3d_3d;
    tf3d_3d.setValue(Tr_l2c_centroid_min3d(0, 0), Tr_l2c_centroid_min3d(0, 1), Tr_l2c_centroid_min3d(0, 2),
    Tr_l2c_centroid_min3d(1, 0), Tr_l2c_centroid_min3d(1, 1), Tr_l2c_centroid_min3d(1, 2),
    Tr_l2c_centroid_min3d(2, 0), Tr_l2c_centroid_min3d(2, 1), Tr_l2c_centroid_min3d(2, 2));
    
    tf::Quaternion tfqt_3d;
    tf3d_3d.getRotation(tfqt_3d);
    tf::Vector3 origin_3d;
    origin_3d.setValue(Tr_l2c_centroid_min3d(0,3),Tr_l2c_centroid_min3d(1,3),Tr_l2c_centroid_min3d(2,3));

    mycalib.transf_3d.setOrigin(origin_3d);
    mycalib.transf_3d.setRotation(tfqt_3d);

    cout << "<<<<< 2.2 calibration via min2D " << endl;
    Eigen::Matrix4d Tr_l2c_centroid_min2d = mycalib.ExtCalib2D(laser_cloud, cam_2d_sorted, Tr_l2c_centroid_min3d);
    cout << "Tr_laser_to_cam_min_2d = " << "\n" << Tr_l2c_centroid_min2d << endl;

    // transfer to TF
    std::vector<double> calib_result_6dof_min2d = eigenMatrix2SixDOF(Tr_l2c_centroid_min2d);
    cout << "x, y, z, roll, pitch, yaw = " << endl;
    for(std::vector<double>::iterator it=calib_result_6dof_min2d.begin(); it<calib_result_6dof_min2d.end(); it++){
        cout  << (*it) << endl;
    } 

    tf::Matrix3x3 tf3d_2d;
    tf3d_2d.setValue(Tr_l2c_centroid_min2d(0, 0), Tr_l2c_centroid_min2d(0, 1), Tr_l2c_centroid_min2d(0, 2),
    Tr_l2c_centroid_min2d(1, 0), Tr_l2c_centroid_min2d(1, 1), Tr_l2c_centroid_min2d(1, 2),
    Tr_l2c_centroid_min2d(2, 0), Tr_l2c_centroid_min2d(2, 1), Tr_l2c_centroid_min2d(2, 2));
    
    tf::Quaternion tfqt_2d;
    tf3d_2d.getRotation(tfqt_2d);
    tf::Vector3 origin_2d;
    origin_2d.setValue(Tr_l2c_centroid_min2d(0,3),Tr_l2c_centroid_min2d(1,3),Tr_l2c_centroid_min2d(2,3));

    mycalib.transf_2d.setOrigin(origin_2d);
    mycalib.transf_2d.setRotation(tfqt_2d);

    ROS_WARN("********** 3.0 calculate error **********"); 
    Eigen::Matrix3d R_min3d, R_min2d;
    R_min3d = Tr_l2c_centroid_min3d.block(0,0,3,3);
    R_min2d = Tr_l2c_centroid_min2d.block(0,0,3,3);

    cout << "<<<<< 3.1 3D Matching Error" << endl;
    vector<double> align_err_min3d = mycalib.calAlignError(mycalib.s1_cloud, mycalib.s2_cloud, Tr_l2c_centroid_min3d);
    vector<double> align_err_min2d = mycalib.calAlignError(mycalib.s1_cloud, mycalib.s2_cloud, Tr_l2c_centroid_min2d);
    cout << "min3d [rmse_x, rmse_y, rmse_z, rmse_total] = [";
    for(auto it : align_err_min3d) cout << it << " ";
    cout << "]" << endl;
    cout << "min2d [rmse_x, rmse_y, rmse_z, rmse_total] = [";
    for(auto it : align_err_min2d) cout << it << " ";
    cout << "]" << endl;

    cout << "<<<<< 3.1 2D Re-projection Error" << endl;
    // min3d
    cv::Mat Tr_l2c_min3d_cv;
    eigen2cv(Tr_l2c_centroid_min2d, Tr_l2c_min3d_cv);
    lv_2d_projected_min3d.clear();
    projectVelo2Cam(mycalib.s1_cloud, cameraMatrix, Tr_l2c_min3d_cv, lv_2d_projected_min3d);
    
    std::vector<double> rmse_2d_reproj_wt_centroid_min3d = calculateRMSE(mycalib.cam_2d_points, lv_2d_projected_min3d);
    cout << "min3d [rmse_2d_reproj_u, rmse_2d_reproj_v, rmse_2d_reproj_total] = \n[";
    for(auto it : rmse_2d_reproj_wt_centroid_min3d) cout << it << " ";
    cout << "]" << endl;

    // min2d
    cv::Mat Tr_l2c_min2d_cv;
    eigen2cv(Tr_l2c_centroid_min2d, Tr_l2c_min2d_cv);
    lv_2d_projected_min2d.clear();
    projectVelo2Cam(mycalib.s1_cloud, cameraMatrix, Tr_l2c_min2d_cv, lv_2d_projected_min2d);

    std::vector<double> rmse_2d_reproj_wt_centroid_min2d = calculateRMSE(mycalib.cam_2d_points, lv_2d_projected_min2d);
    cout << "min2d [rmse_2d_reproj_u, rmse_2d_reproj_v, rmse_2d_reproj_total] = \n[";
    for(auto it : rmse_2d_reproj_wt_centroid_min2d) cout << it << " ";
    cout << "]" << endl;
    
    if(save_calib_file)
    {
        ROS_WARN("********** 4.0 save calibration result **********"); 
        std::ofstream savefile_calib_log;

        savefile_calib_log.open(os_calibfile_log.str().c_str(), ios::out|ios::app);
        cout << "<<<<< opening file " << os_calibfile_log.str() << endl;
        savefile_calib_log << currentDateTime() << "," << ref_ns+"_min3d" << "," << sample_sequence.size();
        for(auto p : calib_result_6dof_min3d){  savefile_calib_log << "," << p;}
        for(int i = 0; i < 9; i++){ savefile_calib_log << "," << R_min3d(i);}
        for(auto p : align_err_min3d){ savefile_calib_log << "," << p;}
        for(auto p : rmse_2d_reproj_wt_centroid_min3d){ savefile_calib_log << "," << p;}
        savefile_calib_log << endl;
        savefile_calib_log.close();

        savefile_calib_log.open(os_calibfile_log.str().c_str(), ios::out|ios::app);
        cout << "<<<<<<<<<< opening file " << os_calibfile_log.str() << endl;
        savefile_calib_log << currentDateTime() << "," << ref_ns+"_min2d" << "," << sample_sequence.size();
        for(auto p : calib_result_6dof_min2d){  savefile_calib_log << "," << p;}
        for(int p = 0; p < 9; p++){ savefile_calib_log << "," << R_min2d(p);}
        for(auto p : align_err_min2d){ savefile_calib_log << "," << p;}
        for(auto p : rmse_2d_reproj_wt_centroid_min2d){ savefile_calib_log << "," << p;}
        savefile_calib_log << endl;
        savefile_calib_log.close();
        
        std::ofstream savefile_exparam;
        savefile_exparam.open(os_extrinsic_min3d.str().c_str(), ios::out);
        cout << "<<<<< opening file " << os_extrinsic_min3d.str() << endl;
        savefile_exparam << "RT_" + ref_ns + "_min3d" << endl;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {                    
                savefile_exparam << Tr_l2c_centroid_min3d(i,j) << ", ";
            }
            savefile_exparam << endl;
        }
        savefile_exparam.close();

        savefile_exparam.open(os_extrinsic_min2d.str().c_str(), ios::out);
        cout << "<<<<< opening file " << os_extrinsic_min2d.str() << endl;
        savefile_exparam << "RT_" + ref_ns + "_min2d" << endl;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {                    
                savefile_exparam << Tr_l2c_centroid_min2d(i,j) << ", ";
            }
            savefile_exparam << endl;
        }
        savefile_exparam.close();

        ROS_WARN("<<<<< calibration result saved!!!");
    }
    return;
}

void RandSampleCalib(int sample_size_, int sample_num_ = 1)
{
    // <<<<<<<< random sample
    list<list<int>> sample_sequence_list;
    int total_num = mycalib.feature_points.size();
    if (total_num == sample_size_)
    {
        sample_num_ = 1;
        cout<< "<<<<< use all " << total_num << " positions to do the extrinsic calibration <<<<<" << endl;
    }
    else
        cout<< "<<<<< use " << sample_num_ << " groups of " << sample_size_ << " positions to do the extrinsic calibration <<<<<" << endl;
   
    RandSample (0, total_num - 1, sample_size_, sample_num_, sample_sequence_list);
   
    if(DEBUG) cout << "sample_sequence_list size = " << sample_sequence_list.size() << endl;
    
    int calib_num = 0;
    int sample_list_size = sample_sequence_list.size();
    for(auto p = sample_sequence_list.begin(); p != sample_sequence_list.end(); p++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr l_cloud_to_calib (new pcl::PointCloud<pcl::PointXYZ>),
                                            c_cloud_to_calib (new pcl::PointCloud<pcl::PointXYZ>);
        vector<cv::Point2f> cam_2d_to_calib;
        if(DEBUG) cout << "sample: ";
        sample_sequence.clear();
        sample_sequence = *p;
        for (auto pt : *p)
        {
            if(DEBUG) cout << pt << " ";
            *l_cloud_to_calib += *(mycalib.feature_points[pt].sensor1_points);
            *c_cloud_to_calib += *(mycalib.feature_points[pt].sensor2_points);
            cam_2d_to_calib.insert(cam_2d_to_calib.end(), mycalib.feature_points[pt].camera_2d.begin(), mycalib.feature_points[pt].camera_2d.end());
        }
        if(DEBUG){
            cout << endl;
            cout << "l_cloud_to_calib: size " << l_cloud_to_calib->size() << endl;
            for(auto pt:l_cloud_to_calib->points){  cout << "[ " << pt.x << ", " << pt.y << ", " << pt.z << " ]" << endl;   }
            cout << "c_cloud_to_calib: size " << c_cloud_to_calib->size() << endl;
            for(auto pt:c_cloud_to_calib->points){  cout << "[ " << pt.x << ", " << pt.y << ", " << pt.z << " ]" << endl;   }
            cout << "cam_2d_to_calib: size " << cam_2d_to_calib.size() << endl;
            for(auto pt : cam_2d_to_calib){ cout  << "[ " << pt.x << ", " << pt.y << " ]" << endl;  }
        }

        calib_num++;
        ROS_INFO("<<<<< Start calibration %d/%d", calib_num, sample_list_size);
        ExtCalib(l_cloud_to_calib, c_cloud_to_calib, cam_2d_to_calib);
    }
    return;
}

void fileHandle()
{
    os_extrinsic_min3d.str("");
    os_extrinsic_min2d.str("");
    os_extrinsic_min3d << calib_result_dir_ << calib_result_name_ << "_exParam_min3d" << ".csv";
    os_extrinsic_min2d << calib_result_dir_ << calib_result_name_ << "_exParam_min2d" << ".csv";

    os_calibfile_log.str("");
    os_calibfile_log << calib_result_dir_ << "L2C_CalibLog.csv";
    if(DEBUG) ROS_INFO("opening %s", os_calibfile_log.str().c_str());
    ifstream check_savefile;
    check_savefile.open(os_calibfile_log.str().c_str(), ios::in); 
    if(!check_savefile)
    {
        if(DEBUG) ROS_INFO("This file doesn't exit!");
        check_savefile.close();
        ofstream of_savefile;
        of_savefile.open(os_calibfile_log.str().c_str());
        
        of_savefile << "time,ref,pos_num,x,y,z,r,p,y,R0,R1,R2,R3,R4,R5,R6,R7,R8,align_err_x,align_err_y,align_err_z,align_err_total,rmse_2d_reproj_u,rmse_2d_reproj_v,rmse_2d_reproj_total" << endl;
        of_savefile.close();
    }
    else
    {
        if(DEBUG) ROS_WARN("This file exits!");
        check_savefile.close();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "extrinsic_calib_l2c");
    ros::NodeHandle nh_("~");

    nh_.param<string>("calib_result_dir_", calib_result_dir_, "");
    nh_.param<string>("camera_info_dir_", camera_info_dir_, "");
    nh_.param<string>("features_info_dir_", features_info_dir_, "");
    nh_.param<string>("calib_result_name_", calib_result_name_, "");
    nh_.param<string>("ns_l", ns_l, "laser");
    nh_.param<string>("ns_c", ns_c, "cam");
    nh_.param("save_calib_file", save_calib_file, false);
    nh_.param("is_multi_exp", is_multi_exp, false);
    nh_.param("is_auto_mode", is_auto_mode, false);
    ref_ns = ns_l + "_to_" + ns_c;
    ostringstream os_in;
    os_in << features_info_dir_;

    if(is_auto_mode)
    {
        bool toStartCalib = false;
        cout << "wait for pattern collection process..." << endl;
        do
        {
            ros::param::get("/end_process", toStartCalib);
        } while (!toStartCalib && ros::ok());
        if(!ros::ok())
        {
            ros::shutdown();
            return 0;
        }
        sleep(2);
    }

    // <<<<<<<<<<<<<<<<<<<<<<<<< laoding data
    ROS_WARN("********** Starat Calibration **********");
    ROS_WARN("********** 1.0 LOADING DATA **********");
    if(mycalib.loadCSV(os_in.str().c_str()))
    {
        std::ostringstream oss_CamIntrinsic;
        oss_CamIntrinsic << camera_info_dir_;
        ParameterReader pr_cam_intrinsic(oss_CamIntrinsic.str()); // ParameterReader is a class defined in "slamBase.h"
        cameraMatrix = pr_cam_intrinsic.ReadMatFromTxt(pr_cam_intrinsic.getData("K"),3,3);
        cv::cv2eigen(cameraMatrix, mycalib.cameraMatrix_);

        fileHandle();
        // <<<<<<<<<<<<<<<<<< random sample to do the calirbation
        int total_pos_num = mycalib.feature_points.size();
        if(!is_multi_exp)
            RandSampleCalib(total_pos_num, 1);
        else// for test
        {
            for(int i = 1; i <= total_pos_num; i++)
            {
                cout << "<<<<< RandSampleCalib " << i << "/" << total_pos_num << " <<<<<" << endl;
                RandSampleCalib(i, total_pos_num);
            }
        }
    }
    return 0;
}