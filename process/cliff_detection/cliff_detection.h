#ifndef _CLIFF_DETECTION_H__
#define _CLIFF_DETECTION_H__

#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include "../p_data.h"
#include "../base.h"

/**
 * 算法步骤：
 * １、直通滤波
 * ２、下采样（重要）
 * ３、地面和障碍物分离（平面拟合或模型滤波）
 * ４、地面边缘提取
 * ５、障碍物点和FOV边缘点Ｋ近邻搜索并去除
*/
namespace NS_CLIFF_DECTECTION
{
    struct STR_X_RANGE
    {
        STR_X_RANGE(int _min,int _max):min(_min),max(_max){}
        float min;
        float max;
    };

    class Cliff_detection:public Object_detect
    {
        public:
            bool init(uint8_t);
            void set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr);
            bool get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);
        private:
            NS_P_DATA::Data p_data;
            pcl::PointCloud<pcl::PointXYZ>::Ptr original_points;        //外部输入点云，不需要初始化
            pcl::PointCloud<pcl::PointXYZ>::Ptr filter_points;          //直通采样后的点
            pcl::PointCloud<pcl::PointXYZ>::Ptr object_points;          //障碍物点
            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points;          //地面点
            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_bound_points;    //地面边缘点
            pcl::PointCloud<pcl::PointXYZ>::Ptr fov_object_points;      //障碍物和fov边缘点
            pcl::PointCloud<pcl::PointXYZ>::Ptr clifff_points;          //干净的边缘点
            void clear_points();
            
            float voxel_x = 10.0;
            float voxel_y = 5.0;
            float voxel_z = 10.0;        //下采样栅格大小
            float object_height = 10;    //距离地面高度mm，障碍物高度
            bool pass_through_voxel();   //直通滤波和下采样
            
            int plane_min_points = 30;
            int plane_threshold = 5;    //地面拟合时的阈值
            std::vector<STR_X_RANGE> x_range;
            bool ground_separate();     //地面和障碍物分离

            int dec_method = 0;
            int k_normal = 5;           //法向量估计点数
            int k_boundary = 250;       //检测边界所需点数
            int mul = 10;
            float angle_threshold = 70; //检测边界点最小角度

            float boundary_radius = 15; //kd搜索领域半径大小
            float cliff_z_max = 300.0;
            bool boundary_filter();     //边缘过滤

    };




}


#endif


