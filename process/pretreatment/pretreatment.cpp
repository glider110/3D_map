#include "pretreatment.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
namespace NS_PRETREATMENT
{
    void Pretreatment::init()
    {
        p_data = NS_P_DATA::Data::get_instence();
    }

    bool Pretreatment::calibration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& intput_points)
    {
        
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients),coefficients_1 (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices),inliers_1 (new pcl::PointIndices);
        
        //尽可能去除非地面信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filter(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (intput_points);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (0,110);
        pass.filter (*pass_filter);
        
        //拟合并提取倾斜地面
        NS_P_DATA::STR_SAC_PALNE_PARAM param(plane_threshold,plane_min_points);
        if(!p_data.sac_plane(pass_filter,coefficients,inliers,param))return false;
        Eigen::Vector3f plane_normal = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
        Eigen::Vector3f vertical = {0.0f, 1.0f, 0.0f};
        if(plane_normal.dot(vertical) < 0)	
        {
            vertical = -1 * vertical;
        }
        r_matrix = calc_rotated_matrix(plane_normal, vertical);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points_t = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (pass_filter);
        extract.setIndices (inliers);
        extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
        extract.filter (*ground_points_t);

        //修正地面，以获得正确fov边缘信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*ground_points_t, *output_points, r_matrix);

        pcl::PointXYZ min,max;
        pcl::getMinMax3D(*output_points,min,max);
        p_data.data.ground_height.min = min.y;
        p_data.data.ground_height.max = max.y;
        
        //获取FOV边缘
        pcl::PointCloud<pcl::PointXYZ>::Ptr fov_boundry_points(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (output_points);
        sor.setLeafSize (voxel_x, voxel_y, voxel_z);
        sor.filter (*output_points);

        NS_P_DATA::STR_SACTTERED_BOUNDARY_PARAM param_b;
        param_b.detect_method = fov_detect_method;
        param_b.k_normal = fov_k_normal;
        param_b.angle_threshold = fov_angle_threshold;
        param_b.k_boundary = fov_k_boundary;
        p_data.scattered_boundary(output_points,fov_boundry_points,param_b);

        pass.setInputCloud (fov_boundry_points);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0,fov_z_max);//fov_z_max
        pass.filter (*p_data.data.FOV_boundary_points);

        if(p_data.data.FOV_boundary_points->points.size() < 10)return false;
        
        return true;

    }

    void Pretreatment::get_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& intput_points
    ,pcl::PointCloud<pcl::PointXYZ>::Ptr& output_points)
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr output_filter_points(new pcl::PointCloud<pcl::PointXYZ>);
        // voxel_filter(intput_points,output_filter_points);
        pcl::transformPointCloud(*intput_points, *output_points, r_matrix);
    }

    void Pretreatment::voxel_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_points
    ,pcl::PointCloud<pcl::PointXYZ>::Ptr& output_points)
    {
        //点云采样
        pcl::VoxelGrid<pcl::PointXYZ> sorVoxel;
        sorVoxel.setInputCloud (input_points);
        sorVoxel.setLeafSize (4.0, 4.0, 7.0);
        sorVoxel.filter (*output_points);

        //点云滤波
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorStaFilter;
        // sorStaFilter.setInputCloud (input_points);
        // sorStaFilter.setMeanK (sta_k);
        // sorStaFilter.setStddevMulThresh (sta_mul);
        // sorStaFilter.filter (*output_points);
        // std::cout << "统计滤波后点云数据点数：" << StatisticalFilter->points.size() << std::endl;
    }


    Eigen::Matrix4f Pretreatment::calc_rotated_matrix(Eigen::Vector3f before, Eigen::Vector3f after)
    {
        before.normalize();
        after.normalize();

        float angle = acos(before.dot(after));
        Eigen::Vector3f p_R = before.cross(after);
        p_R.normalize();

        Eigen::Matrix4f rotateMatrix = Eigen::Matrix4f::Identity();
        rotateMatrix(0, 0) = cos(angle) + p_R[0] * p_R[0] * (1 - cos(angle));
        rotateMatrix(0, 1) = p_R[0] * p_R[1] * (1 - cos(angle) - p_R[2] * sin(angle));
        rotateMatrix(0, 2) = p_R[1] * sin(angle) + p_R[0] * p_R[2] * (1 - cos(angle));

        rotateMatrix(1, 0) = p_R[2] * sin(angle) + p_R[0] * p_R[1] * (1 - cos(angle));
        rotateMatrix(1, 1) = cos(angle) + p_R[1] * p_R[1] * (1 - cos(angle));
        rotateMatrix(1, 2) = -p_R[0] * sin(angle) + p_R[1] * p_R[2] * (1 - cos(angle));

        rotateMatrix(2, 0) = -p_R[1] * sin(angle) + p_R[0] * p_R[2] * (1 - cos(angle));
        rotateMatrix(2, 1) = p_R[0] * sin(angle) + p_R[1] * p_R[2] * (1 - cos(angle));
        rotateMatrix(2, 2) = cos(angle) + p_R[2] * p_R[2] * (1 - cos(angle));

        return rotateMatrix;
    }


}