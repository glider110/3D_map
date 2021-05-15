#ifndef _PRETREATMENT_H__
#define _PRETREATMENT_H__

#include <Eigen/Dense>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../p_data.h"

#define FOV_PATH    "../cliff_33cm.pcd"
namespace NS_PRETREATMENT
{
    class Pretreatment
    {
        public:

            void init();
            //输出转换矩阵和校正后的地面数据
            bool calibration(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
            //输出校正后的点云
            void get_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,pcl::PointCloud<pcl::PointXYZ>::Ptr&);
        private:

            //滤波相关参数
            int sta_k = 20;
            float sta_mul = 1;
            void voxel_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr&);

            //地面拟合相关参数
            float plane_threshold = 2;
            int plane_min_points = 50;
            
            //获取旋转矩阵相关
            Eigen::Matrix4f r_matrix;
            Eigen::Matrix4f calc_rotated_matrix(Eigen::Vector3f, Eigen::Vector3f);

            //提取fov边缘相关参数
            float voxel_x = 15.0;
            float voxel_y = 5.0;
            float voxel_z = 15.0;        //下采样栅格大小
            uint8_t fov_detect_method = 0;      //边界提取方法，0-点数　1-距离
            int fov_k_normal = 9;               //法向量估计点数
            float fov_angle_threshold = 70;      //边界点角度最小值，单位：度
            int fov_k_boundary = 150;            //方法0:边界点k近领点数
            float fov_z_max = 300;              //截取z方向

            NS_P_DATA::Data p_data;
        
    };

}





#endif

