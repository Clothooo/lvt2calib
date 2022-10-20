#ifndef EstimateBoundary_H
#define EstimateBoundary_H

#include <string>
#include <cmath>
#include <math.h>   
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <boost/thread/thread.hpp>


#define DEBUG 0

using namespace std;

template<typename PointT>
class EstimateBoundary
{
    protected:
        double re_ = 30, reforn_ = 50;

    public:
        EstimateBoundary(double re, double reforn): re_(re), reforn_(reforn) {}
        EstimateBoundary(){};
        ~EstimateBoundary(){};
        void setNormEstKSearch(double reforn) { reforn_ = reforn; }
        void setBoundEstKSearch(double re) { re_ = re; }
        void estimateBorders(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::Boundary>::Ptr &boundaries);
};

template<typename PointT>
void EstimateBoundary<PointT>::estimateBorders(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::Boundary>::Ptr &boundaries)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_in, *cloud);
    
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal,pcl::Boundary> boundEst;    // boundary estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;   // noraml estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    normEst.setInputCloud(cloud);
    normEst.setKSearch(reforn_); // number of points to search
    normEst.compute(*normals);
    if(DEBUG)
    {
        std::cout << "reforn_: " << reforn_ << std::endl;
        std::cerr << "normals: " << normals->size() << std::endl;
    }
    
    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normals); //the bounds estimate depends on the normal
    // boundEst.setRadiusSearch(re_); //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
    boundEst.setKSearch(re_);
    boundEst.setAngleThreshold(M_PI / 2); //边界估计时的角度阈值M_PI / 4,并计算k邻域点的法线夹角,若大于阈值则为边界特征点
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    boundEst.compute(*boundaries);

    if(DEBUG)
    {
        std::cerr << "AngleThreshold: " << M_PI / 4 << std::endl;
        cerr << "input cloud size = " << cloud->points.size() << endl;
        cerr << "boundaries.size = " << boundaries->size() << endl;
        std::cerr << "boundaries: " << boundaries->points.size() << std::endl;
    }

    // int boundPointCount = 0;
    // for(auto i = boundaries->begin(); i < boundaries->end(); i++)
    // {
    //     if(i->boundary_point>0)
    //     {
    //     boundPointCount++;
    //     cloud_boundary->push_back(cloud->points[i-boundaries->begin()]);
    //     }
    // }
    // if(DEBUG)
    //     cout << "Count of boundary point: " << boundPointCount << endl;

    return;
}


#endif