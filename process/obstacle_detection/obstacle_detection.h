#ifndef  _OBSTACLE_DETECTION_H_
#define _OBSTACLE_DETECTION_H_

#include "./tool3D/tool3d.h"
#include "./tool3D/customWidget.h"
#include <algorithm>
#include "../base.h"
#define COL 30
#define ROW 20

namespace NS_OBSTACLE_DETECTION
{
        class obstacle_detection:public Object_detect
        {
        public:
                bool init(uint8_t);
                void set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud);
                
                bool get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);

                vector<vector<double>> get_obstacl_index();
                
        private:       
                // void output();
        
                void create_grid();
                void add_grid_properity();
                void merge_grid_feature();
                void identify_obstacle();
                void add_cloud(vector<vector<double>> myvector, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_rct);
                void pass_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

                int column;
                int row;
                int grid_size ;
                Grid grid[ROW][COL];
                int std_count = 0;
                int std_angle = 0;

                pcl::PointCloud<pcl::PointXYZ>::Ptr  boundary_cloud;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rct1;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rct2;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rct3;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_io;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
                pcl::PointCloud<pcl::PointXYZ>::Ptr gridcloud_out;
                pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud_obs;
                pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud_slope;
                pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud_threshold;
                pcl::PointCloud<pcl::PointXYZ>::Ptr calibration_cloud;
                pcl::PointCloud<pcl::PointXYZ>::Ptr  calibration_cloud_after;
                ProcessPointClouds<pcl::PointXYZ> pointCloudXYZProcessor;
                pcl::ExtractIndices<pcl::PointXYZ> extract; //定义一个抽取器


                vector<vector<double>> vector_gridsize;
                vector<vector<double>> vector_grid_y;
                vector<vector<double>> vector_gridstd;
                vector<vector<double>> vector_grid_angle;
                vector<vector<double>> vector_grid_class1;
                vector<vector<double>> vector_grid_class2;
                vector<vector<double>> vector_grid_class3;
                vector<vector<double>> indices_threshold;
                vector<vector<double>> indices_slope;
                vector<vector<double>> indices_obs; // 障碍物
                vector<vector<double>> vector_grid_class_boundary;
        };
}
 #endif
