#include "p_data.h"
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
namespace NS_P_DATA
{
    Data Data::instence;

    STR_DATA_TYPE Data::data;


    Data::Data()
    {
        data.FOV_boundary_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
    }


    Data& Data::get_instence()
    {
        return instence;
    }

    bool Data::sac_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& intput_points
    ,pcl::ModelCoefficients::Ptr& coefficients,pcl::PointIndices::Ptr& inliers
    ,STR_SAC_PALNE_PARAM param)
    {
        if(intput_points->points.size() <3)return false;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (param.threshold);
        
        seg.setInputCloud (intput_points);
        seg.segment (*inliers, *coefficients);
        
        if(param.min_points>0 && param.min_points > inliers->indices.size())
        {
            return false;
        }

        return true;
    }

    bool Data::scattered_boundary(const pcl::PointCloud<pcl::PointXYZ>::Ptr& intput_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_points,STR_SACTTERED_BOUNDARY_PARAM param)
    {
        if(intput_points->points.size()<1)return false;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        float everagedistance =0;
        if(param.detect_method == 1)
        {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
            kdtree.setInputCloud(intput_points);
            int k =2;
            for (int i =0; i < intput_points->size()/2;i++)
            {
                    std::vector<int> nh ;
                    std::vector<float> squaredistance;
                    kdtree.nearestKSearch(intput_points->points[i],k,nh,squaredistance);
                    everagedistance += sqrt(squaredistance[1]);
            }
            everagedistance = everagedistance/(intput_points->size()/2);
        }

        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
        normEst.setInputCloud(intput_points);
        normEst.setSearchMethod(tree);
        normEst.setKSearch(param.k_normal);  //法向估计的点数 
        normEst.compute(*normals);
       
        pcl::BoundaryEstimation<pcl::PointXYZ,pcl::Normal,pcl::Boundary> est;
        pcl::PointCloud<pcl::Boundary> boundaries;
        est.setInputCloud(intput_points);
        est.setInputNormals(normals);
        est.setAngleThreshold(M_PI/180.0*param.angle_threshold);
        est.setSearchMethod (tree);
        if(param.detect_method == 0)
        {
            est.setKSearch(param.k_boundary); 
        }
        else
        {
            est.setRadiusSearch(everagedistance*param.mul_distance);  //搜索半径
        }
        est.compute (boundaries);

        // int countBoundaries = 0;
        
        for (int i=0; i<intput_points->size(); i++)
        {
            uint8_t x = (boundaries.points[i].boundary_point);
            int a = static_cast<int>(x);
            if ( a == 1)
            {
                ( *output_points).push_back(intput_points->points[i]);
                // countBoundaries++;
            }
                
        }

        return true;
        
    }

    
}