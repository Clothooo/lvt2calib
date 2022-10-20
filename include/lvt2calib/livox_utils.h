#ifndef livox_utils_H
#define livox_utils_H

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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

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


using namespace std;
using namespace cv;

void sortPatternCentersYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, vector<pcl::PointXYZ> &v);
pcl::PointXYZ calculateClusterCentroid(std::vector<pcl::PointXYZ> one_acc_points);
cv::Point2f calculateClusterCentroid2d(std::vector<cv::Point2f> one_acc_points);
std::vector<double> calculateRMSE(std::vector<pcl::PointXYZ> ground_truth, std::vector<pcl::PointXYZ> detected);

namespace Livox {
    struct Point
    {
      PCL_ADD_POINT4D; // quad-word XYZ
      float intensity; ///< laser intensity reading
      uint8_t tag; ///< livox tag
      uint8_t line; ///< laser number in lidar
      float range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    }EIGEN_ALIGN16;

    void addRange(pcl::PointCloud<Livox::Point> & pc)
    {
        for(auto pt = pc.points.begin(); pt < pc.points.end(); pt++)
        {
            pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
        }
    }

    struct pointData
    {   // Point Data to save as .pcd
        float x;
        float y;
        float z;
        float intensity;
        float range;
    };

    void writeTitle(const string filename, unsigned long point_num)
    {
        ofstream outfile(filename.c_str(), ios_base::app);// 定位到文件末尾
        if(!outfile)
        {
            cout << "Can't open the file: " << filename << endl;
            exit(0);
        }
        else
        {
            outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
            outfile << "VERSION .7" << endl;
            outfile << "FIELDS x y z intensity range" << endl;
            outfile << "SIZE 4 4 4 4 4" << endl;
            outfile << "TYPE F F F F F" << endl;
            outfile << "COUNT 1 1 1 1 1" << endl;
            outfile << "WIDTH " << to_string(point_num) << endl;
            outfile << "HEIGHT 1" << endl;
            outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
            outfile << "POINTS " << to_string(point_num) << endl;
            outfile << "DATA ascii" << endl;
        }
        ROS_INFO("Save file %s", filename.c_str());
    }

    void writePoingCloud(const string filename, const vector<pointData> singlePCD)
    {
        ofstream outfile(filename.c_str(), ios_base::app);
        if (!outfile) {
            cout << "Can not open the file: " << filename << endl;
            exit(0);
        }
        else
        {
            for(auto i : singlePCD)
            {
                outfile << to_string(i.x) << " " << to_string(i.y) << " " << to_string(i.z) << " " << to_string(i.intensity) << " " << to_string(i.range) << endl;
            }
        }
    }

    void dataSave(string output_path_, int index, const vector<pointData> singlePCD)
    {
        string outputName = output_path_ + to_string(index) + ".pcd";
        writeTitle(outputName, singlePCD.size());
        writePoingCloud(outputName, singlePCD);
    }


    void savePCD(pcl::PointCloud<Livox::Point>::Ptr& lvcloud, int index_, string output_path_)
    {
        vector<pointData> vector_data;
        cout << lvcloud->points.size() << " points needs to be saved" << endl;
        int cloudCount = 0;
        for(auto p = lvcloud->points.begin(); p < lvcloud->points.end(); p++)
        {
            pointData myPoint;
            myPoint.x = p->x;
            myPoint.y = p->y;
            myPoint.z = p->z;
            myPoint.intensity = p->intensity;
            myPoint.range = p->range;

            vector_data.push_back(myPoint);
            cloudCount ++;
        }
        cout << "Saved " << cloudCount << " points" << endl;
        
        dataSave(output_path_, index_, vector_data);
        vector_data.clear();
    }
 


}

POINT_CLOUD_REGISTER_POINT_STRUCT(Livox::Point,
    (float, x, x) 
    (float, y, y) 
    (float, z, z)
    (float, intensity, intensity) 
    (uint8_t, tag, tag) 
    (uint8_t, line, line)
    (float, range, range)
);


#endif