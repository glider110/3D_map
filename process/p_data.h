#ifndef _P_DATA_H__
#define _P_DATA_H__
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>

namespace NS_P_DATA
{
    struct STR_GROUND_HEIGHT
    {
        //注意：该极值为相机坐标系下的极值
        float max;
        float min;
    };
    
    struct STR_DATA_TYPE
    {
        // Eigen::Vector4f ground_coefficient;     //地面系数
        STR_GROUND_HEIGHT ground_height;        //地面高度位置
        pcl::PointCloud<pcl::PointXYZ>::Ptr FOV_boundary_points;
        float machine_height = 0;           //机器最高位置相对３d传感器的坐标
    };

    struct STR_SAC_PALNE_PARAM
    {
        STR_SAC_PALNE_PARAM(){};
        STR_SAC_PALNE_PARAM(float t,int m = 0):threshold(t),min_points(m){};
        float threshold;        //平面拟合阈值
        int min_points;         //平面最少点数设置；<=0表示不设置
    };

    struct STR_SACTTERED_BOUNDARY_PARAM
    {
        STR_SACTTERED_BOUNDARY_PARAM(){};
        STR_SACTTERED_BOUNDARY_PARAM(uint8_t method,int k_n,int k_b,float a,int mul)
        :detect_method(method),k_normal(k_n)
        ,angle_threshold(a),k_boundary(k_b),mul_distance(mul){};
        
        uint8_t detect_method = 0;  //边界提取方法，0-点数　1-距离
        int k_normal;               //法向量估计点数
        float angle_threshold;      //边界点角度最小值，单位：度


        int k_boundary;             //方法0:边界点k近领点数
        int mul_distance;           //方法1:距离乘子
        
    };


    class Data
    {
        public:
            Data();
            static STR_DATA_TYPE data;
            static Data& get_instence();
            bool sac_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
            pcl::ModelCoefficients::Ptr& ,pcl::PointIndices::Ptr&,STR_SAC_PALNE_PARAM);
            bool scattered_boundary(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
            pcl::PointCloud<pcl::PointXYZ>::Ptr&,STR_SACTTERED_BOUNDARY_PARAM);
        private:
            static Data instence;
    };

    

}




#endif

