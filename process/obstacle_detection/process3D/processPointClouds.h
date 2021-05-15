#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <unordered_set>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <pcl/features/boundary.h>
#include <pcl/filters/passthrough.h>
#include "../render/box.h"
//#include "kdtree.h"
#include "../tool3D/kdtree.h"
#include <pcl/visualization/cloud_viewer.h>
template<typename PointT>
class ProcessPointClouds {
    
public:
    ProcessPointClouds();// Constructo

    // ~ProcessPointClouds();  // Destructor

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);


    std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr mysegmentplane(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr &coefficients);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTolerance, bool usePCLClustering=false, int minSize=1, int maxSize=1000);

    typename pcl::PointCloud<PointT>::Ptr mycluster(typename pcl::PointCloud<PointT>::Ptr cloud_filtered, int &cluster_num);

    typename pcl::PointCloud<PointT>::Ptr  myextrctbundary(typename pcl::PointCloud<PointT>::Ptr cloud);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPly(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    typename pcl::PointCloud<PointT>::Ptr passthrough_filter(typename pcl::PointCloud<PointT>::Ptr cloud);

    void statisticalOutlier_filter();

    void show_pointclould(typename pcl::PointCloud<PointT>::Ptr cloud);

    void show_pointclould(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
   

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr process_clould;
        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_pointcloud;
    };

#endif 
