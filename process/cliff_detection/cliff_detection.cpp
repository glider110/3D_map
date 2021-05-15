#include "cliff_detection.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <pcl/segmentation/extract_clusters.h>
namespace NS_CLIFF_DECTECTION
{
    
    bool Cliff_detection::init(uint8_t val)
    {
        p_data = NS_P_DATA::Data::get_instence();
        filter_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        object_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        ground_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        ground_bound_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        clifff_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        fov_object_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
    }

    void Cliff_detection::set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr intput_points)
    {
        original_points = intput_points;
    }

    void Cliff_detection::clear_points()
    {
        filter_points->points.clear();
        object_points->points.clear();
        ground_points->points.clear();
        ground_bound_points->points.clear();
        fov_object_points->points.clear();
        clifff_points->points.clear();
    }


    bool Cliff_detection::get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& out)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_points;
        bool ret_pass,ret_ground,ret_bound,ret_filter;
        ret_pass = ret_ground = ret_bound = ret_filter = false;
        bool is_ok = true;
        static unsigned int i = 0;
        clear_points();
        ret_pass = pass_through_voxel();
        if(ret_pass)
        {
            ret_ground = ground_separate();
            if(ret_ground)
            {
                ret_filter = boundary_filter();
                if(!ret_filter)
                {
                    printf("[%d]缺少地面边缘数据！！！\n",i++);
                    is_ok = false;
                }
            }
            else
            {
                printf("[%d]缺少地面数据！！！\n",i++);
                is_ok = false;
            }
            
        }
        else
        {
            printf("[%d]当前没有输入数据！！！\n",i++);
            is_ok = false;
        }

        if(!is_ok)
        {
            ground_bound_points->points.clear();
        }
        // original_points;        //外部输入点云，不需要初始化
        // filter_points;          //直通采样后的点
        // object_points;          //障碍物点
        // ground_points;          //地面点
        // ground_bound_points;    //地面边缘点
        // fov_object_points;      //障碍物和fov边缘点
        // clifff_points; 
        out.push_back(clifff_points);
    }

    bool Cliff_detection::pass_through_voxel()
    {
        filter_points->points.clear();

        if(original_points->points.size()<2)
        {
            return false;
        }

        //针对地面范围进行截取，去除下斜坡和障碍物平面的影响
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filter(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (original_points);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (p_data.data.ground_height.min-object_height,p_data.data.ground_height.max+10);
        pass.filter (*pass_filter);

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (pass_filter);
        sor.setLeafSize (voxel_x, voxel_y, voxel_z);
        sor.filter (*filter_points);
        return true;
    }

    bool Cliff_detection::ground_separate()
    {
        /**
         * 先把地面和障碍物分割存储：
         * １、使用模型滤波或平面拟合
         * ２、分别存储
        */
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        NS_P_DATA::STR_SAC_PALNE_PARAM param(plane_threshold,plane_min_points);
        if(!p_data.sac_plane(filter_points,coefficients,inliers,param))return false;

        //判断是否为地面平面
        Eigen::Vector3f plane_normal = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
        Eigen::Vector3f vertical = {0.0f, 1.0f, 0.0f};
        if(plane_normal.dot(vertical) < 0)	
        {
            vertical = -1 * vertical;
        }
        if(acos(plane_normal.dot(vertical) > (M_PI/180.0*30)))return false;

        
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (filter_points);
        extract.setIndices (inliers);
        extract.setNegative (false);//提取索引点
        extract.filter (*ground_points);

        extract.setNegative (true); //提取索引之外的点
        extract.filter (*object_points);
        
        x_range.clear();
        if(object_points->points.size()<5)
        {
            // printf("cluster = 0\n");
            return true;
        }
        //对障碍物进行聚类，保存每一个的障碍物群的x范围，后续边界要去除
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (object_points);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (20); // 2cm
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (object_points);
        ec.extract (cluster_indices);
        // printf("cluster = %d\n",cluster_indices.size());
        if(cluster_indices.size()>0)
        {
            
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->push_back ((*object_points)[*pit]); 
                }

                pcl::PointXYZ min,max;
                pcl::getMinMax3D(*cloud_cluster,min,max);
                x_range.push_back(STR_X_RANGE(min.x-10,max.x+10));    //扩大范围，因为实际的边缘点肯定比检测的多
            }

        }

        return true;
    }


    bool Cliff_detection::boundary_filter()
    {
        //边缘提取
        ground_bound_points->points.clear();
        clifff_points->points.clear();
        if(ground_points->points.size()<10)return false;

        //边缘提取
        NS_P_DATA::STR_SACTTERED_BOUNDARY_PARAM param_b;
        param_b.detect_method = dec_method;
        param_b.k_normal = k_normal;
        param_b.angle_threshold = angle_threshold;
        param_b.k_boundary = k_boundary;
        param_b.mul_distance = mul;
        p_data.scattered_boundary(ground_points,ground_bound_points,param_b);
        if(ground_bound_points->points.size()<1)return false;

        //FOV边缘点和障碍物点拼接
        *fov_object_points = *object_points + *p_data.data.FOV_boundary_points;


        //去除障碍物边缘点
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;     
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        kdtree.setInputCloud (ground_bound_points); 
        for(int i(0);i < fov_object_points->points.size();i++)
        {
            if ( kdtree.radiusSearch (fov_object_points->points[i], boundary_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
                for(int j(0);j < pointIdxRadiusSearch.size (); ++j )
                {
                    inliers->indices.push_back(pointIdxRadiusSearch[j]);
                }
                
            }
        }
        std::sort(inliers->indices.begin(),inliers->indices.end());
        inliers->indices.erase(std::unique(inliers->indices.begin(),inliers->indices.end()),inliers->indices.end());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (ground_bound_points);
        extract.setIndices (inliers);
        extract.setNegative (true);         //true:去除cloud中索引为inliers的点；false:只包含索引点
        extract.filter (*clifff_points);

        //截取最远检测距离
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filter(new (pcl::PointCloud<pcl::PointXYZ>));
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (clifff_points);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0,cliff_z_max);
        pass.filter (*clifff_points);
        //去除障碍物造成的阴影边界
        pass.setInputCloud (clifff_points);
        for(int i(0);i<x_range.size();i++)
        {
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (x_range[i].min,x_range[i].max);
            pass.setFilterLimitsNegative (true);
            pass.filter (*clifff_points);
        }

        return true;

    }



}

