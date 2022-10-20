#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <lvt2calib/ClusterCentroids.h>
#include <lvt2calib/Cam2DCircleCenters.h>
#include <lvt2calib/slamBase.h>
#include <lvt2calib/CameraConfig.h>
#include "geometry_msgs/Point.h"

#define DEBUG 0

using namespace std;
using namespace cv;
using namespace Eigen;

int nFrames;
int images_proc_=0;
int images_used_=0;

ros::Publisher cluster_centroids_pub;
ros::Publisher circle_image;
ros::Publisher circle_center_pub, cumulative_pub;
ros::Publisher cam_2d_circle_centers_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud(new pcl::PointCloud<pcl::PointXYZ>);   // Accumulated centers

// Parameters
string image_tp_ = "";  // string image_tp_ = "/camera/left/image_rect_color";
string ns_str = "";
string camera_info_dir_ = "";
int threshold_value_ = 100;
double blob_minCircularity_ = 0.8, blob_minInertiaRatio_ = 0.1, blob_minArea_ = 50;

double  centroid_dis_min_ = 0.15, centroid_dis_max_ = 0.25, center_dis_min_ = 0.25, center_dis_max_= 0.35;
int min_centers_found_ = 4;

bool is_gazebo = false;
bool is_rgb = false;
bool use_darkboard = false;
bool use_morph = false;

string win_raw_img = "1.raw_image", win_undist_img = "2.undistorted_image", win_gray_img = "2.1 gray image", win_circle_img = "3.Draw circle centers on undistorted image"; 

Mat A; 
Mat D;

void load_params()
{
    if (ros::param::get("~image_tp", image_tp_))
    {
        ROS_INFO("Retrived param 'image_tp': %s", image_tp_.c_str()); // color
    }

    if(ros::param::get("~ns_", ns_str))
    {
        ROS_INFO("Retrived param 'ns_': %s", ns_str.c_str());
    }

    if(ros::param::get("~is_rgb", is_rgb))
    {
        ROS_INFO("Retrived param 'is_rgb': %d", is_rgb);
    }

    if(ros::param::get("~is_gazebo", is_gazebo))
    {
        ROS_INFO("Retrived param 'is_gazebo': %d", is_gazebo);
    }

    if(ros::param::get("~use_darkboard", use_darkboard))
    {
        ROS_INFO("Retrived param 'use_darkboard': %d", use_darkboard);
    }

    if(ros::param::get("~use_morph", use_morph))
    {
        ROS_INFO("Retrived param 'use_morph': %d", use_morph);
    }

    if(ros::param::get("~min_centers_found", min_centers_found_))
    {
        ROS_INFO("Retrived param 'min_centers_found': %d", min_centers_found_);
    }

    if(ros::param::get("~centroid_dis_min", centroid_dis_min_))
    {
        ROS_INFO("Retrived param 'centroid_dis_min': %f", centroid_dis_min_);
    }
    
    if(ros::param::get("~centroid_dis_max", centroid_dis_max_))
    {
        ROS_INFO("Retrived param 'centroid_dis_max': %f", centroid_dis_max_);
    }
    
    if(ros::param::get("~center_dis_min", center_dis_min_))
    {
        ROS_INFO("Retrived param 'center_dis_min': %f", center_dis_min_);
    }

    if(ros::param::get("~center_dis_max", center_dis_max_))
    {
        ROS_INFO("Retrived param 'center_dis_max': %f", center_dis_max_);
    }
    
    if(ros::param::get("~camera_info_dir", camera_info_dir_))
    {
        ROS_INFO("Retrived param 'camera_info_dir_': %s", camera_info_dir_.c_str());
        std::ostringstream oss_CamIntrinsic;
        oss_CamIntrinsic << camera_info_dir_;
        ParameterReader pr_cam_intrinsic(oss_CamIntrinsic.str()); // ParameterReader is a class defined in "slamBase.h"
        A = pr_cam_intrinsic.ReadMatFromTxt(pr_cam_intrinsic.getData("K"),3,3);
        D = pr_cam_intrinsic.ReadMatFromTxt(pr_cam_intrinsic.getData("D"),1,5);
    }
}

cv::Mat homography_dlt(const std::vector< cv::Point2f > &x1, const std::vector< cv::Point2f > &x2)
{
  int npoints = (int)x1.size();
  cv::Mat A(2*npoints, 9, CV_64F, cv::Scalar(0));
  // We need here to compute the SVD on a (n*2)*9 matrix (where n is
  // the number of points). if n == 4, the matrix has more columns
  // than rows. The solution is to add an extra line with zeros
  if (npoints == 4)
    A.resize(2*npoints+1, cv::Scalar(0));
  // Since the third line of matrix A is a linear combination of the first and second lines
  // (A is rank 2) we don't need to implement this third line
  for(int i = 0; i < npoints; i++) {      // Update matrix A using eq. 33
    A.at<double>(2*i,3) = -x1[i].x;               // -xi_1
    A.at<double>(2*i,4) = -x1[i].y;               // -yi_1
    A.at<double>(2*i,5) = -1;                     // -1
    A.at<double>(2*i,6) =  x2[i].y * x1[i].x;     //  yi_2 * xi_1
    A.at<double>(2*i,7) =  x2[i].y * x1[i].y;     //  yi_2 * yi_1
    A.at<double>(2*i,8) =  x2[i].y;               //  yi_2
    A.at<double>(2*i+1,0) =  x1[i].x;             //  xi_1
    A.at<double>(2*i+1,1) =  x1[i].y;             //  yi_1
    A.at<double>(2*i+1,2) =  1;                   //  1
    A.at<double>(2*i+1,6) = -x2[i].x * x1[i].x;   // -xi_2 * xi_1
    A.at<double>(2*i+1,7) = -x2[i].x * x1[i].y;   // -xi_2 * yi_1
    A.at<double>(2*i+1,8) = -x2[i].x;             // -xi_2
  }
  // Add an extra line with zero.
  if (npoints == 4) {
    for (int i=0; i < 9; i ++) {
      A.at<double>(2*npoints,i) = 0;
    }
  }
  cv::Mat w, u, vt;
  cv::SVD::compute(A, w, u, vt);
  double smallestSv = w.at<double>(0, 0);
  unsigned int indexSmallestSv = 0 ;
  for (int i = 1; i < w.rows; i++) {
    if ((w.at<double>(i, 0) < smallestSv) ) {
      smallestSv = w.at<double>(i, 0);
      indexSmallestSv = i;
    }
  }
  cv::Mat h = vt.row(indexSmallestSv);
  if (h.at<double>(0, 8) < 0) // tz < 0
    h *=-1;
  cv::Mat _2H1(3, 3, CV_64F);
  for (int i = 0 ; i < 3 ; i++)
    for (int j = 0 ; j < 3 ; j++)
      _2H1.at<double>(i,j) = h.at<double>(0, 3*i+j);
  return _2H1;
}


void pose_from_homography_dlt(const std::vector< cv::Point2f > &xw,
                              const std::vector< cv::Point2f > &pointbuf,
                              cv::Mat &otw, cv::Mat &oRw)
{
  Mat A_inv=A.inv(); 
  cv::Mat oHw = homography_dlt(xw, pointbuf);
  
  cv::Mat h1  = oHw.col(0);
  cv::Mat h2  = oHw.col(1);
  cv::Mat h3  = oHw.col(2);

  cv::Mat c1 = A_inv*h1;
  cv::Mat c2 = A_inv*h2;

  double scale = 1/sqrt(c1.at<double>(0,0)*c1.at<double>(0,0)
                     + c1.at<double>(1,0)*c1.at<double>(1,0)
                     + c1.at<double>(2,0)*c1.at<double>(2,0));

  cv::Mat r1=scale*c1;
  cv::Mat r2=scale*c2;
  cv::Mat r3=r1.cross(r2);

  cv::Mat r1_t, r2_t, r3_t;
  cv::Mat r1_, r2_, t_;
  r1_ = c1/scale;
  r2_ = c2/scale;
  t_ = A_inv*h3/scale;
  cv::transpose(r1_, r1_t);
  cv::transpose(r2_, r2_t);
  cv::transpose(t_, r3_t);

  otw = scale*A_inv*h3;
  for(int i=0; i < 3; i++) {
    oRw.at<double>(i,0) = r1.at<double>(i,0);
    oRw.at<double>(i,1) = r2.at<double>(i,0);
    oRw.at<double>(i,2) = r3.at<double>(i,0);
  }
}


//sort//
bool order_X(const Vec2f &p1, const Vec2f &p2)
{
  return p1.val[0] < p2.val[0];
}

bool order_Y(const Vec2f &p1, const Vec2f &p2)
{
  return p1.val[1] < p2.val[1];
}
//sort//


void image_process(cv::Mat original_image, const sensor_msgs::ImageConstPtr& image_msg)
{
    images_proc_++;
    cv::Mat undistorted_image, gray;
    cv::undistort(original_image, undistorted_image, A, D, A);
    namedWindow(win_raw_img);
    cv::imshow(win_raw_img, original_image);
    namedWindow(win_undist_img);
    cv::imshow(win_undist_img, undistorted_image);
    cv::waitKey(1);
    cv::Mat image_copy;
    image_copy = undistorted_image.clone();
    
    cv::Size boardSize;
    boardSize.height = 2;
    boardSize.width = 2;

    bool found = false;
    std::vector<cv::Point2f>  pointbuf;  // coordinates of centers in image frame
    
    if(is_rgb)    
    {
        // If in simulation, do gray-scale and binarization.
        if(is_rgb && DEBUG) cout << "[" << ns_str << "] is RGB!" << endl;
        // to gray
        cv::Mat image_gray;
        cv::cvtColor(undistorted_image, image_gray, CV_RGB2GRAY);
        namedWindow(win_gray_img);
        imshow(win_gray_img, image_gray);

        image_copy = image_gray;

        waitKey(10);
    }

    // find circles
    // Setup SimpleBlobDetector parameters. 
    // spot detection
    SimpleBlobDetector::Params params;
    // Filter by Area.
    // Area in pixels
    params.filterByArea = true;
    params.minArea = blob_minArea_;
    params.maxArea = 1000000000;
    // Filter by Inertia.
    params.filterByInertia = true;
    params.minInertiaRatio = blob_minInertiaRatio_;
    // Filter by circularity.
    params.filterByCircularity = true;
    params.minCircularity = blob_minCircularity_;
    // Filter by color (default as 0, black)
    if(use_darkboard)
    {
        params.blobColor = 255;
    }
   
    ROS_INFO("[%s] Detecting circles......", ns_str.c_str());
    #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
    
        // Set up detector with params
        SimpleBlobDetector detector(params);
        
        // You can use the detector this way
        // detector.detect( im, keypoints);
    
    #else
    
        // Set up detector with params
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        found = findCirclesGrid(image_copy, boardSize, pointbuf, CALIB_CB_SYMMETRIC_GRID+CALIB_CB_CLUSTERING, detector);

    #endif

    if(found) 
    {
        ROS_INFO("[%s] Find circles!", ns_str.c_str());
        //sort//
        sort(pointbuf.begin(), pointbuf.end(), order_Y);
        sort(pointbuf.begin(), pointbuf.begin() + 2, order_X);
        sort(pointbuf.begin() + 2, pointbuf.begin() + 4, order_X);
        if(DEBUG)
        {
            cout << "pointbuf: " << pointbuf  << endl;
        }

        drawChessboardCorners( undistorted_image, boardSize, Mat(pointbuf), found ); 
        namedWindow(win_circle_img);
        imshow(win_circle_img, undistorted_image);
        cv::waitKey(10);

        sensor_msgs::ImagePtr circle_ros = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted_image).toImageMsg();
        circle_image.publish(circle_ros);
        
        // world coordinate
        std::vector<cv::Point2f> xw;
        xw.push_back(Point2f(300.0f,0.0f));
        xw.push_back(Point2f(300.0f,300.0f));
        xw.push_back(Point2f(0.0f,0.0f));
        xw.push_back(Point2f(0.0f,300.0f));

        // Initialize translation and rotation matrix
        cv::Mat otw = Mat::zeros(3, 1, CV_64F); // Translation vector
        cv::Mat oRw = Mat::eye(3, 3, CV_64F); // Rotation matrix
        pose_from_homography_dlt(xw, pointbuf, otw, oRw);
        if(DEBUG)
        {
            cout<<"Translation_Matrix="<<endl<<otw<<endl;
            cout<<"Rotation_Matrix="<<endl<<oRw<<endl;
        }
        

        // Get transformation from camera to board
    	cv::Mat RT_b2c, Tr_b2c;  // from board to camera
        cv::hconcat(oRw, otw, RT_b2c);
        cv::Mat temp_row = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 1.0 );
        cv::vconcat(RT_b2c, temp_row, Tr_b2c);

        cv::Mat Tr_c2b;  // from camera to board
        Tr_c2b = Tr_b2c.inv();
        if(DEBUG) cout << "Tr_c2b = " << Tr_c2b << endl;


        // Four centers in world frame
        std::vector<cv::Point3d> wX ;
        wX.push_back( cv::Point3d(  0, 0, 0) ); // wX_0 (-L, -L, 0)^T
        wX.push_back( cv::Point3d(  300, 0, 0) ); // wX_1 ( L, -L, 0)^T
        wX.push_back( cv::Point3d(  0, 300, 0) ); // wX_2 ( L,  L, 0)^T
        wX.push_back( cv::Point3d(  300, 300, 0) ); // wX_3 (-L,  L, 0)^T
 
        std::vector<cv::Point3d> points_3d;
        for(int i = 0; i < wX.size(); i++) 
        {
            cv::Mat oX_test = oRw*cv::Mat(wX[i]) + otw; // Update oX, oY, oZ
            points_3d.push_back( cv::Point3d( oX_test.at<double>(0, 0),
                                    oX_test.at<double>(1,0),oX_test.at<double>(2,0) )); 
                                    //  = (oX/oZ, oY/oZ)
        }
        // cv::waitKey(0);


        pcl::PointCloud<pcl::PointXYZ>::Ptr  final_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
                                            four_center_pc(new pcl::PointCloud<pcl::PointXYZ>);
        if(DEBUG) cout << "points_3d: " << endl;
        for(int i=0; i<points_3d.size(); i++)
        {
            pcl::PointXYZ p;
            p.x=points_3d[i].x/1000.0;
            p.y=points_3d[i].y/1000.0;
            p.z=points_3d[i].z/1000.0;
            if(DEBUG) cout << "[" << p.x << ", " << p.y << ", " << p.z << "]" <<endl;
            
            four_center_pc->push_back(p);
        }

        // compute the centroid of four center_pt detected
        pcl::PointXYZ center_centroid;
        float accx = 0, accy = 0, accz = 0;
        for(auto it : four_center_pc->points)
        {
            accx += it.x;
            accy += it.y;
            accz += it.z;
        }
        center_centroid.x = accx/four_center_pc->points.size();
        center_centroid.y = accy/four_center_pc->points.size();
        center_centroid.z = accz/four_center_pc->points.size();
        if(DEBUG) ROS_INFO("Centroid %f %f %f", center_centroid.x, center_centroid.y, center_centroid.z);

        // compute the distance from each center to the centroid
        std::vector< std::vector<float> > found_centers;
        bool valid = true;
        for(auto center_pt = four_center_pc->points.begin(); center_pt < four_center_pc->points.end(); ++center_pt)
        {
            double centroid_distance = sqrt(pow(fabs(center_centroid.x-center_pt->x),2) + pow(fabs(center_centroid.y-center_pt->y),2));
            // if(DEBUG)   ROS_INFO("Center [%f, %f] Distance to centroid %f, should be in (%.2f, %.2f)", center_pt->x, center_pt->y, centroid_distance, centroid_dis_min_, centroid_dis_max_);

            if (centroid_distance < centroid_dis_min_)
            {
                // if(DEBUG) ROS_INFO("centroid_distance < centroid_dis_min_ !!");
                if(DEBUG)   ROS_INFO("Invalid! Center [%f, %f] Distance to centroid %f, should be in (%.2f, %.2f)", center_pt->x, center_pt->y, centroid_distance, centroid_dis_min_, centroid_dis_max_);
                valid = false;
            }
            else if(centroid_distance > centroid_dis_max_)
            {
                // if(DEBUG) ROS_INFO("centroid_distance > centroid_dis_max_ !!");
                if(DEBUG)   ROS_INFO("Invalid! Center [%f, %f] Distance to centroid %f, should be in (%.2f, %.2f)", center_pt->x, center_pt->y, centroid_distance, centroid_dis_min_, centroid_dis_max_);
                valid = false;
            }
            else
            {
                for(std::vector<std::vector <float> >::iterator it = found_centers.begin(); it != found_centers.end(); ++it) 
                {
                    float dis = sqrt(pow(fabs((*it)[0]-center_pt->x),2) + pow(fabs((*it)[1]-center_pt->y),2));
                    // if(DEBUG) ROS_INFO("%f", dis);
                    // if (dis < 0.25 || dis >0.35){
                    if (dis < center_dis_min_ || dis > center_dis_max_){
                        if(DEBUG) ROS_INFO("Invalid! The dis %f from center [%f, %f, %f] to center[%f, %f, %f] out of range!", dis, center_pt->x, center_pt->y, center_pt->z, (*it)[0], (*it)[1], (*it)[2]);
                        valid = false;
                        break;
                    }
                }
            }
            if(valid)
            {
                if(DEBUG) 
                {
                    ROS_INFO("Valid circle found!");
                    cout << "circle center_pt: (" << center_pt->x << ", " << center_pt->y << ", " << center_pt->z << ")" << endl;
                }
                std::vector<float> found_center;
                found_center.push_back(center_pt->x);
                found_center.push_back(center_pt->y);
                found_center.push_back(center_pt->z);
                found_centers.push_back(found_center);
            }
        }

        if(found_centers.size() >= min_centers_found_)
        {
            ROS_INFO("[%s] Enough centers: %d", ns_str.c_str(), found_centers.size());
            for (auto it = found_centers.begin(); it < found_centers.end(); ++it)
            {
                pcl::PointXYZ center;
                center.x = (*it)[0];
                center.y = (*it)[1];
                center.z = (*it)[2];
                final_cloud->points.push_back(center);
            }
            *cumulative_cloud+=*final_cloud;

            sensor_msgs::PointCloud2 ros_pointcloud;
            pcl::toROSMsg(*cumulative_cloud, ros_pointcloud);   // accumulative centers
            ros_pointcloud.header = image_msg->header;
            cumulative_pub.publish(ros_pointcloud);   // Topic: /lvt2calib/cumulative_cloud

            nFrames++;
            images_used_=nFrames;

            sensor_msgs::PointCloud2 final_ros;
            pcl::toROSMsg(*final_cloud,final_ros);      // single frame center
            final_ros.header.stamp = ros::Time::now();
            final_ros.header.frame_id="camera";
            circle_center_pub.publish(final_ros);    // Topic: /lvt2calib/circle_center_cloud

            lvt2calib::ClusterCentroids center_cloud;
            center_cloud.header.stamp = ros::Time::now();
            center_cloud.total_iterations = images_proc_;
            center_cloud.cluster_iterations = images_used_;
            center_cloud.header.frame_id = "camera";
            center_cloud.cloud = final_ros;

            cluster_centroids_pub.publish(center_cloud);      //Topic：/lvt2calib/centers_cloud


            // Publish four 2D circle centers (unit: pixel)
            lvt2calib::Cam2DCircleCenters cam_2d_circle_centers_msg;
            cam_2d_circle_centers_msg.points.clear();

            int i = 0;
            for (std::vector<cv::Point2f>::iterator it = pointbuf.begin(); it != pointbuf.end(); ++it) {
                geometry_msgs::Point point;
                point.x = (*it).x;
                point.y = (*it).y;
                point.z = 0;
                cam_2d_circle_centers_msg.points.push_back(point);
                i++;
            }

            cam_2d_circle_centers_msg.header = image_msg->header;
            if(DEBUG)
            {
                ROS_INFO("There are %d 2D circle centers (unit: pixel)", cam_2d_circle_centers_msg.points.size());
                for(auto i : cam_2d_circle_centers_msg.points)
                {
                    cout << "[ " << i.x << ", " << i.y << ", " << i.z << " ]" << endl;
                }
            }
            cam_2d_circle_centers_pub.publish(cam_2d_circle_centers_msg);   //Topic：/lvt2calib/cam_2d_circle_center
        }
        else
        {
            ROS_WARN("[%s] Not enough centers: %d", ns_str.c_str(), found_centers.size());
        }
        // Clear the 4 centers on image plane
        pointbuf.clear();
        // cv::waitKey(10);

        // cout<<"****************************************[End of callback!]*****************************************"<<endl;
    }
    else
    {
        ROS_WARN("[%s] Can't find the circles, continue!", ns_str.c_str());
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
     
    ROS_INFO("[%s] Processing image...", ns_str.c_str());

    cv::Mat thermal_image;
    try
    {
        thermal_image=cv_bridge::toCvShare(msg,"bgr8")->image;
        if(DEBUG) ROS_INFO("[%s] Image loaded successfully!", ns_str.c_str());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("[%s] Could not convert from '%s' to 'bgr8'.", ns_str.c_str(), msg->encoding.c_str());
    }
    
    image_process(thermal_image, msg);
}

void param_callback(lvt2calib::CameraConfig &config, uint32_t level)
{
    blob_minCircularity_ = config.minCircularity;
    blob_minInertiaRatio_ = config.minInertiaRatio;
    blob_minArea_ = config.minArea;
    ROS_INFO("New blob min circularity: %f", blob_minCircularity_);
    ROS_INFO("New blob min inertia ratio: %f", blob_minInertiaRatio_);
    ROS_INFO("New blob min area: %f", blob_minArea_);
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"cam_pattern");
    ros::NodeHandle nh("~");
    load_params();

    circle_image=nh.advertise<sensor_msgs::Image> ("image/circle_center",1);
    circle_center_pub=nh.advertise<sensor_msgs::PointCloud2> ("circle_center_cloud", 1);
    cumulative_pub=nh.advertise<sensor_msgs::PointCloud2> ("cumulative_cloud", 1);
    cluster_centroids_pub = nh.advertise<lvt2calib::ClusterCentroids> ("centers_cloud",1);
    cam_2d_circle_centers_pub = nh.advertise<lvt2calib::Cam2DCircleCenters>("cam_2d_circle_center", 1);

    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(image_tp_, 1, imageCallback);

    dynamic_reconfigure::Server<lvt2calib::CameraConfig> server;
    dynamic_reconfigure::Server<lvt2calib::CameraConfig>::CallbackType f;
    f = boost::bind(param_callback, _1, _2);
    server.setCallback(f);
    ROS_INFO("initialized...");
    

    ros::Rate loop_rate(30);
    bool end_process = false;
    bool pause_process = false;
    ros::param::set("/cam_paused", false);
    while(ros::ok())
    {
        ros::param::get("/end_process", end_process);
        ros::param::get("/pause_process", pause_process);
        if(end_process) 
            break;
        if(pause_process)
        {
            ROS_WARN("<<<<<<<<<<<< [%s] PAUSE <<<<<<<<<<<<", ns_str.c_str());
            cumulative_cloud -> clear();
            ros::param::set("/cam_paused", true);
            while(pause_process && !end_process && ros::ok())
            {
                ros::param::get("/end_process", end_process);
                ros::param::get("/pause_process", pause_process);
            }
            ros::param::set("/cam_paused", false);
            destroyWindow(win_circle_img);

            if(end_process)
                break;
            images_proc_ = 0;
            images_used_ = 0;
        }
        else    
            ros::spinOnce();
    }

    ROS_WARN("<<<<<<<<<<<< [%s] END <<<<<<<<<<<<", ns_str.c_str());
    destroyAllWindows();
    ros::shutdown();
    return 0;
}