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

#include <lvt2calib/lvt2Calib.h>
#include <lvt2calib/slamBase.h>

#define DEBUG 0

using namespace std;

string calib_result_dir_ = "", features_info_dir_ = "", calib_result_name_ = "";
string ns_l1, ns_l2, ref_ns;

bool save_calib_file = false, is_multi_exp = false;
bool is_auto_mode = false;
vector<pcl::PointXYZ> lv1_3d_for_reproj;
std::vector<cv::Point2f> lv_2d_projected, lv_2d_projected_min2d, lv_2d_projected_min3d,
                        cam_2d_for_reproj;
int sample_size = 0, sample_num = 0, sample_size_min = 0, sample_size_max = 0;
ostringstream os_extrinsic_min3d, os_calibfile_log;
list<int> sample_sequence;

lvt2Calib mycalib(L2L_CALIB);

void ExtCalib(pcl::PointCloud<pcl::PointXYZ>::Ptr laser1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr laser2_cloud)
{
    ROS_WARN("********** 2.0 calibration start **********");
    cout << "<<<<< calibration via min3D " << endl;
    // min3D Calibration
    Eigen:: Matrix4d Tr_L2toL1_centroid_min_3d = mycalib.ExtCalib3D(laser1_cloud, laser2_cloud);
    Eigen::Matrix4d Tr_L1toL2_centroid_min3d = Tr_L2toL1_centroid_min_3d.inverse(); 
    cout << "Tr_L1_to_L2_centroid_min_3d = " << "\n" << Tr_L1toL2_centroid_min3d << endl;

    // transfer to TF
    std::vector<double> calib_result_6dof_min3d = eigenMatrix2SixDOF(Tr_L1toL2_centroid_min3d);
    cout << "x, y, z, roll, pitch, yaw = " << endl;
    for(std::vector<double>::iterator it=calib_result_6dof_min3d.begin(); it<calib_result_6dof_min3d.end(); it++){
        cout  << (*it) << endl;
    } 

    tf::Matrix3x3 tf3d_3d;
    tf3d_3d.setValue(Tr_L1toL2_centroid_min3d(0, 0), Tr_L1toL2_centroid_min3d(0, 1), Tr_L1toL2_centroid_min3d(0, 2),
    Tr_L1toL2_centroid_min3d(1, 0), Tr_L1toL2_centroid_min3d(1, 1), Tr_L1toL2_centroid_min3d(1, 2),
    Tr_L1toL2_centroid_min3d(2, 0), Tr_L1toL2_centroid_min3d(2, 1), Tr_L1toL2_centroid_min3d(2, 2));
    
    tf::Quaternion tfqt_3d;
    tf3d_3d.getRotation(tfqt_3d);
    tf::Vector3 origin_3d;
    origin_3d.setValue(Tr_L1toL2_centroid_min3d(0,3),Tr_L1toL2_centroid_min3d(1,3),Tr_L1toL2_centroid_min3d(2,3));

    mycalib.transf_3d.setOrigin(origin_3d);
    mycalib.transf_3d.setRotation(tfqt_3d);

    Eigen::Matrix3d R_min3d;
    R_min3d = Tr_L1toL2_centroid_min3d.block(0,0,3,3);

    ROS_WARN("********** 3.0 calculate error **********");   
    cout << "<<<<< 3D Matching Error" << endl;
    vector<double> align_err = mycalib.calAlignError(mycalib.s1_cloud, mycalib.s2_cloud, Tr_L1toL2_centroid_min3d);
    cout << "[rmse_x, rmse_y, rmse_z, rmse_total] = [";
    for(auto it : align_err) cout << it << " ";
    cout << "]" << endl;
    
    if(save_calib_file)
    {
        ROS_WARN("********** 4.0 save result **********");   
        std::ofstream savefile_calib_log;

        savefile_calib_log.open(os_calibfile_log.str().c_str(), ios::out|ios::app);
        cout << "<<<<< opening file " << os_calibfile_log.str() << endl;
        savefile_calib_log << currentDateTime() << "," << ref_ns << "," << sample_sequence.size();
        // for(auto p : sample_sequence){  of_calib_file << p << " ";  }
        for(auto p : calib_result_6dof_min3d){  savefile_calib_log << "," << p;}
        for(int p = 0; p < 9; p++){ savefile_calib_log << "," << R_min3d(p);}
        for(auto p : align_err){ savefile_calib_log << "," << p;}
        savefile_calib_log << endl;
        savefile_calib_log.close();
        
        std::ofstream of_calib_file;
        of_calib_file.open(os_extrinsic_min3d.str().c_str(), ios::out);
        cout << "<<<<< opening file " << os_extrinsic_min3d.str() << endl;
        of_calib_file << "RT_" + ref_ns << endl;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                of_calib_file << Tr_L1toL2_centroid_min3d(i,j) << ",";
            }
            of_calib_file << endl;
        }
        of_calib_file.close();

        ROS_WARN("<<<<< calibration result saved!!!");
    }
    return;
}

void RandSampleCalib(int sample_size_, int sample_num_ = 1)
{
    // <<<<<<< random sample
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr l1_cloud_to_calib (new pcl::PointCloud<pcl::PointXYZ>),
                                            l2_cloud_to_calib (new pcl::PointCloud<pcl::PointXYZ>);
        if(DEBUG) cout << "sample: ";
        sample_sequence.clear();
        sample_sequence = *p;
        for (auto pt : *p)
        {
            if(DEBUG) cout << pt << " ";
            *l1_cloud_to_calib += *(mycalib.feature_points[pt].sensor1_points);
            *l2_cloud_to_calib += *(mycalib.feature_points[pt].sensor2_points);
        }
        if(DEBUG){
            cout << endl;
            cout << "l1_cloud_to_calib: size " << l1_cloud_to_calib->size() << endl;
            for(auto pt:l1_cloud_to_calib->points){  cout << "[ " << pt.x << ", " << pt.y << ", " << pt.z << " ]" << endl;   }
            cout << "l2_cloud_to_calib: size " << l2_cloud_to_calib->size() << endl;
            for(auto pt:l2_cloud_to_calib->points){  cout << "[ " << pt.x << ", " << pt.y << ", " << pt.z << " ]" << endl;   }
        }
        
        calib_num++;
        ROS_INFO("<<<<< Start calibration %d/%d", calib_num, sample_list_size);
        ExtCalib(l1_cloud_to_calib, l2_cloud_to_calib);
    }
    return;
}

void fileHandle()
{
    os_extrinsic_min3d.str("");
    os_extrinsic_min3d << calib_result_dir_ << calib_result_name_ << "_exParam" << ".csv";
    os_calibfile_log.str("");
    os_calibfile_log << calib_result_dir_ << "L2L_CalibLog.csv";
    if(DEBUG) ROS_INFO("opening %s", os_calibfile_log.str().c_str());
    ifstream check_savefile;
    check_savefile.open(os_calibfile_log.str().c_str(), ios::in); 
    if(!check_savefile)
    {
        if(DEBUG) ROS_INFO("This file doesn't exit!");
        check_savefile.close();
        ofstream of_savefile;
        of_savefile.open(os_calibfile_log.str().c_str());
        of_savefile << "time,ref,pos_num,x,y,z,r,p,y,R0,R1,R2,R3,R4,R5,R6,R7,R8,align_err_x,align_err_y,align_err_z,align_err_total" << endl;
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
    ros::init(argc, argv, "extrinsic_calib_l2l");
    ros::NodeHandle nh_("~");

    nh_.param<string>("calib_result_dir_", calib_result_dir_, "");
    nh_.param<string>("features_info_dir_", features_info_dir_, "");
    nh_.param<string>("calib_result_name_", calib_result_name_, "");
    nh_.param<string>("ns_l1", ns_l1, "laser1");
    nh_.param<string>("ns_l2", ns_l2, "laser2");
    nh_.param("save_calib_file", save_calib_file, false);
    nh_.param("is_multi_exp", is_multi_exp, false);
    nh_.param("is_auto_mode", is_auto_mode, false);
    ref_ns = ns_l1 + "_to_" + ns_l2;
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

    // <<<<<<<<<<<<<< laoding data
    ROS_WARN("********** Starat Calibration **********");
    ROS_WARN("********** 1.0 LOADING DATA **********");
    if(mycalib.loadCSV(os_in.str().c_str()))
    {
        fileHandle();
        // <<<<<<< random sample to do the calirbation
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