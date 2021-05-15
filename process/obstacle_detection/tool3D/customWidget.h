#ifndef _CUSTOMWIDGET_H_
#define _CUSTOMWIDGET_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <string>
#include <fstream>
#include  "../process3D/processPointClouds.h"
#include "../process3D/processPointClouds.cpp"

#define PI 3.141592
using  namespace std;

struct myVect3
{
        double x;
        double y;
        double z;
};

void show(vector<vector<int>> &v);

void show1(vector<vector<double>> &v);

void show1_chw(vector<vector<double>> &v,int& total);

void show2(vector<vector<int>> &v);

void extrcBoundary_1(vector<vector<double>> vectorBoundary, int type_value, vector<vector<double>> &boudary_merge1, vector<vector<double>> &boudary_merge3);

vector<vector<int>> extrcBoundary(vector<vector<double>> vectorBoundary, int value, pcl::PointCloud<pcl::PointXYZ>::Ptr &boundary_cloud, vector<vector<double>> &boudary_merge);

vector<vector<int>> extrcBoundary_obs(vector<vector<double>> vectorBoundary, int value1 , int value2, pcl::PointCloud<pcl::PointXYZ>::Ptr &boundary_cloud, vector<vector<double>> &boudary_merge);

int statisticsValueNumbers(vector<vector<double>> vectorBoundary, int value);

void savedata(vector<vector<double>> &v);

double calAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

float_t get_y_mean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

float_t get_y_std(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

bool get_Angle(myVect3 v1, myVect3 v2, double &angle_abs);

void get_norm(pcl::ModelCoefficients::Ptr coefficients, myVect3 &floor_norm, myVect3 &incline_norm);

 class Grid
{
public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud;
        pcl::PointIndices::Ptr grid_inliers{new pcl::PointIndices};
        //bool low_emp = true;
        double m_mean_y ;
        double m_std_y;
        double m_gradient;
        double m_angle;

        void grid_clear()
        {
                grid_inliers->indices.clear();
             //   grid_cloud->points.clear();

        }
};



#endif