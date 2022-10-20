#ifndef FourCircleCenters_H
#define FourCircleCenters_H

#define PCL_NO_RECOMPILE
#define DEBUG 0
// #define STATIC_ANALYSE 1

#include <string>
#include <cmath>
#include <math.h>   
#include <unistd.h>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/console/time.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace Eigen;


class FourCircleCenters
{
    private:
        double circle_seg_thre_ = 0.02, circle_radius_ = 0.12, centroid_dis_min_ = 0.15, centroid_dis_max_ = 0.25; 
        int min_centers_found_ = 4; 

    public:
        FourCircleCenters(){};
        ~FourCircleCenters(){};

        void setCircleSegDistanceThreshold(double threshold)
        {
            circle_seg_thre_ = threshold;
        }
        void setCircleRadius(double radius)
        {
            circle_radius_ = radius;
        }
        void setCentroidDis(double min, double max)
        {
            centroid_dis_min_ = min;
            centroid_dis_max_ = max;
        }
        void setMinNumCentersFound(int num)
        {
            min_centers_found_ = num;
        }

        bool FindFourCenters(pcl::PointCloud<pcl::PointXYZI>::Ptr &calib_boundary_, pcl::PointCloud<pcl::PointXYZI>::Ptr &circle_center_cloud_, Eigen::Matrix4f Tr_tpl2ukn);
        bool FindFourCenters(pcl::PointCloud<pcl::PointXYZI>::Ptr &calib_boundary_, pcl::PointCloud<pcl::PointXYZI>::Ptr &circle_center_cloud_);
};



bool FourCircleCenters::FindFourCenters(pcl::PointCloud<pcl::PointXYZI>::Ptr &calib_boundary_, pcl::PointCloud<pcl::PointXYZI>::Ptr &circle_center_cloud_, Eigen::Matrix4f Tr_tpl2ukn)
{
    bool find_centers = false;
    // *************** Extract circles ******************
    pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_circle (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> circle_segmentation;
    circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
    circle_segmentation.setDistanceThreshold (circle_seg_thre_);
    circle_segmentation.setMethodType(pcl::SAC_RANSAC);
    circle_segmentation.setOptimizeCoefficients(true);
    circle_segmentation.setMaxIterations(1000);
    circle_segmentation.setRadiusLimits(circle_radius_ - circle_seg_thre_, circle_radius_ + circle_seg_thre_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr four_cneters_registed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr four_cneters_back(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZI>); // Used for removing inliers
    pcl::copyPointCloud<pcl::PointXYZI>(*calib_boundary_, *copy_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZI>); // To store circle points
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZI>); // To store circle points
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>); // Temp pc used for swaping

    pcl::PointXYZI edges_centroid; //  点云中心点
    float accx = 0., accy = 0.;
    for(auto it = copy_cloud->points.begin(); it < copy_cloud->points.end(); it++)
    {
        accx+=it->x;
        accy+=it->y;
        it->z=0.0;
    }

    // Compute and add center to clouds
    edges_centroid.x =  accx/copy_cloud->points.size();
    edges_centroid.y =  accy/copy_cloud->points.size();
    edges_centroid.z =  0.0;
    if(DEBUG) ROS_INFO("Centroid %f %f %f", edges_centroid.x, edges_centroid.y, edges_centroid.z);


    // pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::copyPointCloud(*copy_cloud, *copy_cloud2);
    // for(auto it:copy_cloud2->points)
    // {
    //     cout << it.z << endl;
    // }
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    std::vector< std::vector<float> > found_centers;
    std::vector<pcl::PointXYZI> centroid_cloud_inliers;
    bool valid = true;   // if it is a valid center

    while ((copy_cloud->points.size() + centroid_cloud_inliers.size()) > 3 && found_centers.size() < 4 && copy_cloud->points.size())    
    {
        circle_segmentation.setInputCloud (copy_cloud);
        circle_segmentation.segment (*inliers_circle, *coefficients_circle);  //when doing circle segmentation, coefficients means the x-y coordinate of the found circle
        if(DEBUG)   cout << "inliers_circle->indices.size() = " << inliers_circle->indices.size() << endl; //by yao
        if (inliers_circle->indices.size () == 0)   break;

        // Extract the inliers
        extract.setInputCloud (copy_cloud);
        extract.setIndices (inliers_circle);
        extract.setNegative (false);
        extract.filter (*circle_cloud);

        // Add center point to cloud
        pcl::PointXYZI center;
        center.x = *coefficients_circle->values.begin();
        center.y = *(coefficients_circle->values.begin()+1);
        center.z = 0;
        // Make sure there is no circle at the center of the pattern or far away from it
       

        double centroid_distance = sqrt(pow(fabs(edges_centroid.x-center.x),2) + pow(fabs(edges_centroid.y-center.y),2));
        if(DEBUG) 
        {
            ROS_INFO("Center [%f, %f] Distance to centroid %f, should be in (%.2f, %.2f)", center.x, center.y, centroid_distance, centroid_dis_min_, centroid_dis_max_);
        }
        if (centroid_distance < centroid_dis_min_)
        {
            if(DEBUG) ROS_INFO("centroid_distance < centroid_dis_min_ !!");
            valid = false;
        // ???
            for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = circle_cloud->points.begin(); pt < circle_cloud->points.end(); ++pt){
                centroid_cloud_inliers.push_back(*pt);
            }
        }
        else if(centroid_distance > centroid_dis_max_){
            valid = false;
            if(DEBUG) ROS_INFO("centroid_distance > centroid_dis_max_ !!");

        }else{
            if(DEBUG) ROS_INFO("Valid centroid");
            for(std::vector<std::vector <float> >::iterator it = found_centers.begin(); it != found_centers.end(); ++it) 
            {
                float dis = sqrt(pow(fabs((*it)[0]-center.x),2) + pow(fabs((*it)[1]-center.y),2));
                if(DEBUG) ROS_INFO("%f", dis);
                if (dis < 0.25){
                    valid = false;
                    break;
                }
            }

            // If center is valid, check if any point from wrong_circle belongs to it, and pop it if true
            // ???
            for (std::vector<pcl::PointXYZI>::iterator pt = centroid_cloud_inliers.begin(); pt < centroid_cloud_inliers.end(); ++pt)
            {
                pcl::PointXYZI schrodinger_pt;
                schrodinger_pt.x = pt->x;   schrodinger_pt.y = pt->y;   schrodinger_pt.z = pt->z;
                double distance_to_cluster = sqrt(pow(schrodinger_pt.x-center.x,2) + pow(schrodinger_pt.y-center.y,2) + pow(schrodinger_pt.z-center.z,2));
                // if(DEBUG) ROS_INFO("Distance to cluster: %lf", distance_to_cluster);
                if(distance_to_cluster<circle_radius_+0.02){
                centroid_cloud_inliers.erase(pt);
                --pt; // To avoid out of range
                }
            }
        }

         if (valid){
            if(DEBUG) 
            {
                ROS_INFO("Valid circle found");
                cout << "circle center: (" << center.x << ", " << center.y << ", " << center.z << ")" << endl;
            }
            std::vector<float> found_center;
            found_center.push_back(center.x);
            found_center.push_back(center.y);
            found_center.push_back(center.z);
            found_centers.push_back(found_center);
            // if(DEBUG) ROS_INFO("Remaining points in cloud %lu", copy_cloud->points.size());
        }

        // Remove inliers from pattern cloud to find next circle
        extract.setNegative (true);
        extract.filter(*cloud_f);
        copy_cloud.swap(cloud_f);
        valid = true;

        if(DEBUG) ROS_INFO("Remaining points in cloud %lu", copy_cloud->points.size());
    }
    
    Eigen::Affine3f translation;
    translation.matrix() = Tr_tpl2ukn;
    // cout << "Tr in findfourcircle = \n" << Tr_tpl2ukn << endl;
    if(found_centers.size() >= min_centers_found_ && found_centers.size() < 5)
    {
        for (std::vector<std::vector<float> >::iterator it = found_centers.begin(); it < found_centers.end(); ++it)
        {
            pcl::PointXYZI center;
            center.x = (*it)[0];
            center.y = (*it)[1];
            center.z = (*it)[2];
            if(DEBUG)
            {
                pcl::PointXYZRGB center_rgb;
                pcl::copyPoint(center, center_rgb);
                center_rgb.r = center_rgb.g = center_rgb.b = 0;
                switch (it-found_centers.begin())
                {
                case 0:
                    center_rgb.r = 255;
                    break;
                case 1:
                    center_rgb.g = 255;
                    break;
                case 2:
                    center_rgb.b = 255;
                    break;
                case 3:
                    center_rgb.r = center_rgb.b = 255;
                    break;                
                default:
                    break;
                }
                four_cneters_registed->points.push_back(center_rgb);
            }
            pcl::PointXYZI center_rotated_back = pcl::transformPoint(center, translation);
            // center_rotated_back.x = (- coefficients->values[1] * center_rotated_back.y - coefficients->values[2] * center_rotated_back.z - coefficients->values[3])/coefficients->values[0];
            // cumulative_cloud->push_back(center_rotated_back);
            circle_center_cloud_->push_back(center_rotated_back);
        }
        if(DEBUG) pcl::transformPointCloud(*four_cneters_registed, *four_cneters_back, translation);
        

        if(DEBUG) 
        {
            cout << "Four circle centers:" << endl;
            for(auto p:circle_center_cloud_->points)
            {
                cout << "[" << p.x << ", " << p.y << ", " << p.z << "]" << endl;
            }
        }
        find_centers = true;
        ROS_INFO("[Laser] Found enough centers");
    }else{
        ROS_WARN("[Laser] Not enough centers: %ld", found_centers.size());

        for (std::vector<std::vector<float> >::iterator it = found_centers.begin(); it < found_centers.end(); ++it)
        {
            pcl::PointXYZI center;
            center.x = (*it)[0];
            center.y = (*it)[1];
            center.z = (*it)[2];
            center.intensity = it - found_centers.begin();
           
            circle_center_cloud_->push_back(center);
        }
        pcl::transformPointCloud(*circle_center_cloud_, *circle_center_cloud_, translation);


        return find_centers;
    }

    circle_cloud.reset();
    copy_cloud.reset(); // Free memory
    cloud_f.reset(); // Free memory

    return find_centers;
}

bool FourCircleCenters::FindFourCenters(pcl::PointCloud<pcl::PointXYZI>::Ptr &calib_boundary_, pcl::PointCloud<pcl::PointXYZI>::Ptr &circle_center_cloud_)
{
    bool find_centers = false;
    // *************** Extract circles ******************
    pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_circle (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> circle_segmentation;
    circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE3D);
    circle_segmentation.setDistanceThreshold (circle_seg_thre_);
    circle_segmentation.setMethodType(pcl::SAC_RANSAC);
    circle_segmentation.setOptimizeCoefficients(true);
    circle_segmentation.setMaxIterations(1000);
    circle_segmentation.setRadiusLimits(circle_radius_ - circle_seg_thre_, circle_radius_ + circle_seg_thre_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr four_cneters_registed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr four_cneters_back(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZI>); // Used for removing inliers
    pcl::copyPointCloud<pcl::PointXYZI>(*calib_boundary_, *copy_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZI>); // To store circle points
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZI>); // To store circle points
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>); // Temp pc used for swaping

    pcl::PointXYZI edges_centroid; //  点云中心点
    float accx = 0., accy = 0., accz= 0.;
    for(auto it = copy_cloud->points.begin(); it < copy_cloud->points.end(); it++)
    {
        accx+=it->x;
        accy+=it->y;
        accz+=it->z;
    }

    // Compute and add center to clouds
    edges_centroid.x =  accx/copy_cloud->points.size();
    edges_centroid.y =  accy/copy_cloud->points.size();
    edges_centroid.z =  accz/copy_cloud->points.size();;
    if(DEBUG) ROS_INFO("Centroid %f %f %f", edges_centroid.x, edges_centroid.y, edges_centroid.z);


    // pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::copyPointCloud(*copy_cloud, *copy_cloud2);
    // for(auto it:copy_cloud2->points)
    // {
    //     cout << it.z << endl;
    // }
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    std::vector< std::vector<float> > found_centers;
    std::vector<pcl::PointXYZI> centroid_cloud_inliers;
    bool valid = true;   // if it is a valid center

    while ((copy_cloud->points.size() + centroid_cloud_inliers.size()) > 3 && found_centers.size() < 4 && copy_cloud->points.size())    
    {
        circle_segmentation.setInputCloud (copy_cloud);
        circle_segmentation.segment (*inliers_circle, *coefficients_circle);  //when doing circle segmentation, coefficients means the x-y coordinate of the found circle
        if(DEBUG)   cout << "inliers_circle->indices.size() = " << inliers_circle->indices.size() << endl; //by yao
        if (inliers_circle->indices.size () == 0)   break;

        // Extract the inliers
        extract.setInputCloud (copy_cloud);
        extract.setIndices (inliers_circle);
        extract.setNegative (false);
        extract.filter (*circle_cloud);

        // Add center point to cloud
        pcl::PointXYZI center;
        center.x = *coefficients_circle->values.begin();
        center.y = *(coefficients_circle->values.begin()+1);
        center.z = *(coefficients_circle->values.begin()+2);
        // Make sure there is no circle at the center of the pattern or far away from it
       

        double centroid_distance = sqrt(pow(fabs(edges_centroid.x-center.x),2) + pow(fabs(edges_centroid.y-center.y),2) + + pow(fabs(edges_centroid.z-center.z),2));
        if(DEBUG) 
        {
            ROS_INFO("Center [%f, %f, %f] Distance to centroid %f, should be in (%.2f, %.2f)", center.x, center.y, center.z, centroid_distance, centroid_dis_min_, centroid_dis_max_);
        }
        if (centroid_distance < centroid_dis_min_)
        {
            if(DEBUG) ROS_INFO("centroid_distance < centroid_dis_min_ !!");
            valid = false;
        // ???
            for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = circle_cloud->points.begin(); pt < circle_cloud->points.end(); ++pt){
                centroid_cloud_inliers.push_back(*pt);
            }
        }
        else if(centroid_distance > centroid_dis_max_){
            valid = false;
            if(DEBUG) ROS_INFO("centroid_distance > centroid_dis_max_ !!");

        }else{
            if(DEBUG) ROS_INFO("Valid centroid");
            for(std::vector<std::vector <float> >::iterator it = found_centers.begin(); it != found_centers.end(); ++it) 
            {
                float dis = sqrt(pow(fabs((*it)[0]-center.x),2) + pow(fabs((*it)[1]-center.y),2) + pow(fabs((*it)[2]-center.z),2));
                if(DEBUG) ROS_INFO("%f", dis);
                if (dis < 0.25){
                    valid = false;
                    break;
                }
            }

            // If center is valid, check if any point from wrong_circle belongs to it, and pop it if true
            // ???
            if(DEBUG)   cout << "In schrodinger_pt" << endl;
            for (std::vector<pcl::PointXYZI>::iterator pt = centroid_cloud_inliers.begin(); pt < centroid_cloud_inliers.end(); ++pt)
            {
                // if(DEBUG)   cout << "In schrodinger_pt" << endl;
                pcl::PointXYZI schrodinger_pt;
                schrodinger_pt.x = pt->x;   schrodinger_pt.y = pt->y;   schrodinger_pt.z = pt->z;
                double distance_to_cluster = sqrt(pow(schrodinger_pt.x-center.x,2) + pow(schrodinger_pt.y-center.y,2) + pow(schrodinger_pt.z-center.z,2));
                // if(DEBUG) ROS_INFO("Distance to cluster: %lf", distance_to_cluster);
                if(distance_to_cluster<circle_radius_+0.02){
                centroid_cloud_inliers.erase(pt);
                --pt; // To avoid out of range
                }
            }
        }

         if (valid){
            if(DEBUG) 
            {
                ROS_INFO("Valid circle found");
                cout << "circle center: (" << center.x << ", " << center.y << ", " << center.z << ")" << endl;
            }
            std::vector<float> found_center;
            found_center.push_back(center.x);
            found_center.push_back(center.y);
            found_center.push_back(center.z);
            found_centers.push_back(found_center);
            // if(DEBUG) ROS_INFO("Remaining points in cloud %lu", copy_cloud->points.size());
        }

        // Remove inliers from pattern cloud to find next circle
        extract.setNegative (true);
        extract.filter(*cloud_f);
        copy_cloud.swap(cloud_f);
        valid = true;

        if(DEBUG) ROS_INFO("Remaining points in cloud %lu", copy_cloud->points.size());
    }
    
    // Eigen::Affine3f translation;
    // translation.matrix() = Tr_tpl2ukn;
    // cout << "Tr in findfourcircle = \n" << Tr_tpl2ukn << endl;
    if(found_centers.size() >= min_centers_found_ && found_centers.size() < 5)
    {
        for (std::vector<std::vector<float> >::iterator it = found_centers.begin(); it < found_centers.end(); ++it)
        {
            pcl::PointXYZI center;
            center.x = (*it)[0];
            center.y = (*it)[1];
            center.z = (*it)[2];
            if(DEBUG)
            {
                pcl::PointXYZRGB center_rgb;
                pcl::copyPoint(center, center_rgb);
                center_rgb.r = center_rgb.g = center_rgb.b = 0;
                switch (it-found_centers.begin())
                {
                case 0:
                    center_rgb.r = 255;
                    break;
                case 1:
                    center_rgb.g = 255;
                    break;
                case 2:
                    center_rgb.b = 255;
                    break;
                case 3:
                    center_rgb.r = center_rgb.b = 255;
                    break;                
                default:
                    break;
                }
                four_cneters_registed->points.push_back(center_rgb);
            }
            // pcl::PointXYZI center_rotated_back = pcl::transformPoint(center, translation);
            // center_rotated_back.x = (- coefficients->values[1] * center_rotated_back.y - coefficients->values[2] * center_rotated_back.z - coefficients->values[3])/coefficients->values[0];
            // cumulative_cloud->push_back(center_rotated_back);
            circle_center_cloud_->push_back(center);
        }
        // if(DEBUG) pcl::transformPointCloud(*four_cneters_registed, *four_cneters_back, translation);
        

        cout << "Four circle centers:" << endl;
        for(auto p:circle_center_cloud_->points)
        {
            cout << "[" << p.x << ", " << p.y << ", " << p.z << "]" << endl;
        }
        find_centers = true;
    }else{
        ROS_WARN("[Laser] Not enough centers: %ld", found_centers.size());

        for (std::vector<std::vector<float> >::iterator it = found_centers.begin(); it < found_centers.end(); ++it)
        {
            pcl::PointXYZI center;
            center.x = (*it)[0];
            center.y = (*it)[1];
            center.z = (*it)[2];
            center.intensity = it - found_centers.begin();
           
            circle_center_cloud_->push_back(center);
        }

        return find_centers;
    }

    circle_cloud.reset();
    copy_cloud.reset(); // Free memory
    cloud_f.reset(); // Free memory

    return find_centers;
}




#endif