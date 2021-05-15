#include "slop_down.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
namespace NS_SLOP_DOWN
{
    bool Slop_down::init(uint8_t val)
    {
        boundary_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    }

    void Slop_down::set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr intput_points)
    {
        original_points = intput_points;
    }

    bool Slop_down::get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& out)
    {
        process();
        out.push_back(boundary_points);
    }

    void Slop_down::process()
    {
        //当前点数过少返回空
        if(original_points->points.size() < 50)return;

        //裁剪：地面高度往下截100mm
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filter(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (original_points);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (p_data.data.ground_height.max+5,p_data.data.ground_height.max+threshold_ground_down);
        pass.filter (*pass_filter);

        //裁剪后点数过少返回空
        if(pass_filter->points.size() < 50)return;


        //拟合并提取倾斜地面
        NS_P_DATA::STR_SAC_PALNE_PARAM param(5,100);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        if(!p_data.sac_plane(pass_filter,coefficients,inliers,param))return;

        //通过角度判断当前是否是下斜坡
        Eigen::Vector3f plane_normal = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
        Eigen::Vector3f vertical = {0.0f, -1.0f, 0.0f};
        if(plane_normal.dot(vertical) < 0)	
        {
            plane_normal = -1 * plane_normal;
        }
        if(plane_normal[2] < 0)     //非下斜坡
        {
            printf(" plane_normal[2] = %f 非下斜坡!\n",plane_normal[2]);
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points_t = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (pass_filter);
        extract.setIndices (inliers);
        extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
        extract.filter (*ground_points_t);

        if(ground_points_t->points.size()<10)return;


        //边缘提取
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_bound_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        NS_P_DATA::STR_SACTTERED_BOUNDARY_PARAM param_b;
        param_b.detect_method = 0;
        param_b.k_normal = 5;
        param_b.angle_threshold = 70;
        param_b.k_boundary = 100;
        p_data.scattered_boundary(ground_points_t,ground_bound_points,param_b);
        if(ground_bound_points->points.size()<1)return;


        //斜面边缘裁剪：地面高度往下截30mm
        pass.setInputCloud (ground_bound_points);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (p_data.data.ground_height.max,p_data.data.ground_height.max+30);
        pass.filter (*boundary_points);



    }






}