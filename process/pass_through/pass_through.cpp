#include "pass_through.h"
#include <pcl/filters/passthrough.h>
namespace NS_PASS_THROUGH
{

    bool Pass_through::init(uint8_t val)
    {
        boundary_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        
    }

    void Pass_through::set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr intput_points)
    {
        original_points = intput_points;
    }


    bool Pass_through::get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& out)
    {
        boundary_points->points.clear();

        if(original_points->points.size() < 50)return false;  //输入点过少直接返回
        
        //裁剪：z机器高度往上截threshold_dt_heightmm
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (original_points);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (p_data.data.machine_height-threshold_dt_height,p_data.data.machine_height);
        pass.filter (*boundary_points);

        // if(boundary_points->points.size() < 50)return;  //裁剪后点过少直接返回

        out.push_back(boundary_points);
    
    }




}