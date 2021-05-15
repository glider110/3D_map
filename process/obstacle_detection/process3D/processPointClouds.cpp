#include "processPointClouds.h"
//#include <pcl/kdtree/kdtrcustomWidgetee.h>
// Constructor

#include <boost/thread.hpp>
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


// // Destructor
// template<typename PointT>
// ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    #if 0
    // Voxel size should be large enough to help speed up the processing but not so large that object definition is completely lost
    // TOTRY: setting camera angles in environment.cpp can help pick a good region of interest. E.g. set the camera to have a top down or side overview.

    auto startTime = std::chrono::steady_clock::now();
    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
  

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);  // using filterRes as leaf size
    voxelGrid.filter(*filteredCloud);  // filteredCloud is a pointer, so we dereference it

        // Region-based filtering
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBox(true);  // true because dealing with points inside cropBox
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*croppedCloud);  // saving results in croppedCloud
    std::cout << " VoxelGrid.filter: " << filteredCloud->size() << endl;
    std::cout << " cropBox.filter: " << croppedCloud->size() << endl;
    // Roof-point filtering
    // Using a PCL CropBox to find the roof point indices and then feeding those indices to a PCL ExtractIndices object to remove them (similar to the way the segmentation algorithm extracts points)
    
    // std::vector<int> indices;

    // pcl::CropBox<PointT> roof(true);
    // roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    // roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    // roof.setInputCloud(croppedCloud);
    // roof.filter(indices);

    // pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // for (int point : indices)
    //     inliers->indices.push_back(point);

    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(croppedCloud);
    // extract.setIndices(inliers);  // indices of roof points
    // extract.setNegative(true);  // removing points
    // extract.filter(*croppedCloud);  // extracting those indices from inliers
    //std::cout << " cropBox1111.filter: " << croppedCloud->size() - inliers->indices.size()<< endl;
    // TODO: use renderBox to visualise the area where the ego car's roof points were contained
    // TOTRY: use the renderBox function to figure out how big boxes will look in the scene

    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "Point cloud filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedCloud;
    #endif
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filter(new (pcl::PointCloud<pcl::PointXYZ>));
    // // Create the filtering object
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud (original_points);
    // pass.setFilterFieldName ("y");
    // pass.setFilterLimits ();
    // pass.filter (*pass_filter);

    // pass.setInputCloud (pass_filter);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits ();
    // pass.filter (*pass_filter);

}


template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::cout << "Total cloud->points.size(): " << cloud->points.size() << std::endl;

	std::unordered_set<int> inliersResult;  // starts as 0
	srand(time(NULL));

	// For max iterations
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--) // > 0
	{
		std::unordered_set<int> inliers;  // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
		while(inliers.size() <= 3)
			inliers.insert(rand()%(cloud->points.size()));  // using modulo, value between 0 and the size of cloud  // inliers will hold the index of points

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		std::cout << "inliers.size(): " << inliers.size() << std::endl;
		auto itr = inliers.begin();  // pointer to the beginning of inliers
		x1 = cloud->points[*itr].x;  // checking what value it is by dereferencing the pointer
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
		itr++;  // incrementing the iterator by one
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // 3D
        double u1, u2, u3, v1, v2, v3;
        u1 = (x2 - x1);
        u2 = (y2 - y1);
        u3 = (z2 - z1);
        v1 = (x3 - x1);
        v2 = (y3 - y1);
        v3 = (z3 - z1);
        // v1 [3] = {u1, u2, u3};
        // v2 [3] = {v1, v2, v3};

        double i, j, k;
        i = u2 * v3 - v2 * u3;
        j = v1 * u3 - u1 * v3;
        k = u1 * v2 - v1 * u2;

        // crossProd [3] = {i, j, k};

        double a, b, c, d;
		a = i;
		b = j;
		c = k;
        d = -1.0 * (i * x1 + j * y1 + k * z1);

		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (inliers.count(i) > 0)  // if point is one of the two points that make the line
				continue;

			pcl::PointXYZ point = cloud->points[i];
			std::cout << "Point: " << point << std::endl;

			float x3 = point.x;  // member x from point
			float y3 = point.y;
            float z3 = point.z;

			double distance = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c); 
			
			std::cout << "Distance: " << distance << std::endl;

			if (distance <= distanceTol)
				inliers.insert(i);
		}

		std::cout << "Fitted line: inliers.size(): " << inliers.size() << std::endl;

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr groundCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
    {
        groundCloud->points.push_back(cloud->points[index]);
    }

    // Creating filtering object and extracting inliers
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);  // dereferencing pointer
    std::cerr << "PointCloud representing the planar component: " << cloud->width * cloud->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, groundCloud);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::cout << "Total cloud->points.size(): " << cloud->points.size() << std::endl;  // 20
	std::unordered_set<int> inliersResult;  // starts as 0
	srand(time(NULL));
	
	// For max iterations
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted plane
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--)  // > 0
	{
		std::unordered_set<int> inliers;  // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
		while(inliers.size() <= 3)
			inliers.insert(rand()%(cloud->points.size())); // using modulo, value between 0 and the size of cloud  // inliers will hold the index of points

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();  // pointer to the beginning of inliers
		x1 = cloud->points[*itr].x;  // checking value by dereferencing pointer
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
		itr++;  // increment the iterator by one
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // 3D
        double u1, u2, u3, v1, v2, v3;
        u1 = (x2 - x1);
        u2 = (y2 - y1);
        u3 = (z2 - z1);
        v1 = (x3 - x1);
        v2 = (y3 - y1);
        v3 = (z3 - z1);
        // v1 [3] = {u1, u2, u3};
        // v2 [3] = {v1, v2, v3};

        double i, j, k;
        i = u2 * v3 - v2 * u3;
        j = v1 * u3 - u1 * v3;
        k = u1 * v2 - v1 * u2;

        // crossProd [3] = {i, j, k};

        double a, b, c, d;
		a = i;
		b = j;
		c = k;
        d = -1.0 * (i * x1 + j * y1 + k * z1);  // for 3D

		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (inliers.count(i) > 0)  // if point is one of the two points that make the line
				continue;

			PointT point = cloud->points[i];
			float x3 = point.x;  // member x from point
			float y3 = point.y;
            float z3 = point.z;

			double distance = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c);

			if (distance <= distanceThreshold)
				inliers.insert(i);
		}

        if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

    std::unordered_set<int> inliers = inliersResult;

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new typename pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];

		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl;
	std::cout << "cloudInliers->points.size(): " << cloudInliers->points.size() << std::endl;
	std::cout << "cloudOutliers->points.size(): " << cloudOutliers->points.size() << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = cloudOutliers;
    segResult.second = cloudInliers;

    // TONOTE: not using SeparateClouds function at all

	return segResult;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::mysegmentplane(typename pcl::PointCloud<PointT>::Ptr cloud, pcl::ModelCoefficients::Ptr &coefficients)
{

    pcl::PCDWriter writer;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                //设置聚类的内点索引
    //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //平面模型的因子
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); //分割模型
    seg.setMethodType(pcl::SAC_RANSAC);    //随机参数估计方法
    seg.setMaxIterations(100);             //最大的迭代的次数
    seg.setDistanceThreshold(200);        //设置阀值

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    //bool flag = (inliers->indices.size() > 0)?true:false;
   //cout<<flag<<endl;
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        //return NULL;
    }
    // 从输入的点云中提取平面模型的内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers); //提取内点的索引并存储在其中
    extract.setNegative(false);

    // 得到与平面表面相关联的点云数据
    extract.filter(*cloud_plane);
    // std::cout << "平面分割之前: " << cloud->points.size() << " data points." << std::endl;
    // std::cout << "平面分割之后: " << cloud_plane->points.size() << " data points." << std::endl;

    // std::stringstream ss1;
    // ss1 << "cloud_plane.pcd";
    // writer.write<pcl::PointXYZ>(ss1.str(), *cloud_plane, false);
    // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    return cloud_plane;
}


template<typename PointT>
void euclideanClusteringHelper(int index, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTolerance)  // & cluster (works), & tree (doesn't)
{
    // If we have never processed this point before
    std::cout << "\tI" << index << " : " << processed[index] << std::endl;
    processed[index] = true;

    cluster.push_back(index);
    std::vector<int> nearby = tree->search(index, distanceTolerance);

    int unprocessed_points = 0;
    for (int id : nearby) {
        if (processed[id] != true) {
            unprocessed_points += 1; 
        }
    }  
    std::cout << "\t\t" << unprocessed_points << "/" << nearby.size() << " : " << " left to process." << std::endl;

    for (int id : nearby) {
        if (processed[id] != true) {
            euclideanClusteringHelper(id, cloud, cluster, processed, tree, distanceTolerance);
        }
    }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)  // TODO: should tree be passed by reference
{
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(cloud->points.size(), false);  // same size as points, each of which starts as false

	int i = 0;
	while (i < cloud->points.size())
	{
		if (processed[i] == false)
		{
            // If the point has not been processed, create a new cluster
            std::vector<int> newIdCluster;
            std::cout << "\nCreating a new cluster - Index " << i << std::endl;
            euclideanClusteringHelper(i, cloud, newIdCluster, processed, tree, distanceTol);  // i: point id, cluster passed by reference

            typename pcl::PointCloud<PointT>::Ptr newPointCluster(new typename pcl::PointCloud<PointT>());
            std::cout << "\nCreating Cluster\n" << std::endl;
            for (int id : newIdCluster)
            {
                std::cout << "\tAdding " << "(" << cloud->points[id].x << ", " << cloud->points[id].y << ", " << cloud->points[id].z << ")" << std::endl;
                newPointCluster->points.push_back((cloud->points[id]));
            }
            clusters.push_back(newPointCluster);  // Assertion failed: (px != 0), function operator->, file /usr/local/include/boost/smart_ptr/shared_ptr.hpp, line 734. // Abort trap: 6
		}
		i++;
	}

	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTolerance, bool usePCLClustering, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;  // vector of point clouds

    if (usePCLClustering == true)  // use built-in PCL euclidean-clustering functions
    {
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);  // KdTree object for optimised search during extraction
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(distanceTolerance);  // 2cm  //  PCL example uses 0.02
        ec.setMinClusterSize(minSize);  // PCL example uses 100
        ec.setMaxClusterSize(maxSize);  // PCL example uses 25000
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);

        for (pcl::PointIndices getIndices : clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

            for (int index : getIndices.indices)
                cloudCluster->points.push_back(cloud->points[index]);

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;
            clusters.push_back(cloudCluster);

            std::cout << "PointCloud representing the cluster: " << cloudCluster->points.size() << " data points." << std::endl;
        }  
    }
    else  // use custom clustering algorithm
    {
        KdTree<PointT>* tree3D = new KdTree<PointT>(cloud);  // on stack: KdTree<PointT> tree3D;  // TOCHECK: difference between () and no ()

        for (int i = 0; i < cloud->points.size(); i++) 
            tree3D->insertPointIndex(i);
        std::cout << "\nKD Tree Built - Size : " << cloud->points.size() << "\n" << std::endl;

        clusters = euclideanClustering(cloud, tree3D, distanceTolerance);
        std::cout << "\nClusters Found : " << clusters.size() << "\n" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Obstacle clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::mycluster(typename pcl::PointCloud<PointT>::Ptr cloud_filtered,int &cluster_num)
{
    std::cout << "进入分分割接口" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;                           //vector<Pointindics>和上述的平面分割区别,平面分割只有最大的分割平面          提取点云可以extracindics和遍历索引pushback进去
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //欧式聚类对象
    ec.setClusterTolerance(200);                      // 设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize(500);                        //设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize(50000);                       //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree);                          //设置点云的搜索机制
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中
    cluster_num = cluster_indices.size();
    cout << " cluster_num" << cluster_num<<endl;
     pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_max(new pcl::PointCloud<pcl::PointXYZ>);
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    { //迭代容器中的点云的索引，并且分开保存索引的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            //设置保存点云的属性问题
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
        if (j == 0)
        {
            *cloud_cluster_max = *cloud_cluster;
            std::stringstream ss0;
            ss0 << "cloud_cluster_max.pcd";
            writer.write<pcl::PointXYZ>(ss0.str(), *cloud_cluster_max, false);
        }
        j++;
        *add_cloud += *cloud_cluster;
        pcl::io::savePCDFileASCII("add_cloud.pcd", *add_cloud);
    }
    return cloud_cluster_max;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::myextrctbundary(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_bundary(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
    normEst.setInputCloud(cloud);
    normEst.setSearchMethod(tree_bundary);
    //normEst.setRadiusSearch(2);  //法向估计的半径
    normEst.setKSearch(9); //法向估计的点数
    normEst.compute(*normals);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud(cloud_filtered);
    //cout << "normal size is " << normals->size() << endl;
    //边界评估需要点云
    est.setInputCloud(cloud);
    est.setInputNormals(normals);  /*M_PI_2 */
    est.setAngleThreshold(M_PI_2); ///在这里 由于构造函数已经对其进行了初始化 为Π/2 ，必须这样 使用 M_PI/2  M_PI_2
    est.setSearchMethod(tree);
    est.setKSearch(100); //一般这里的数值越高，最终边界识别的精度越好 20+
    //  est.setRadiusSearch(everagedistance);  //搜索半径
    est.compute(boundaries);

    //  pcl::PointCloud<pcl::PointXYZ> boundPoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
    int countBoundaries = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);
        int a = static_cast<int>(x); //该函数的功能是强制类型转换
        if (a == 1)
        {

            //  boundPoints.push_back(cloud->points[i]);
            (*boundPoints).push_back(cloud->points[i]);
            countBoundaries++;
        }
        else
            noBoundPoints.push_back(cloud->points[i]);
    }
    std::cout << "boudary size is：" << countBoundaries << std::endl;
    //  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);
    //pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
    //pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);
    return boundPoints;
}


    template <typename PointT>
    Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1)
        PCL_ERROR("Couldn't read file \n");
    std::cerr << "Loaded " << cloud->points.size () << " data points from " + file << std::endl;

    return cloud;
}


template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPly(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPLYFile<PointT>(file, *cloud) == -1)


        PCL_ERROR("Couldn't read file \n");
    std::cout << "读入ply文件....." << std::endl;
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    sort(paths.begin(), paths.end());  // sorting files in ascending order so playback is chronological

    return paths;
}

//typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::passthrough_filter();

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT> ::passthrough_filter(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    //typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x; //设置滤波器对象
    pass_x.setInputCloud(cloud);      //设置输入点云
    pass_x.setFilterFieldName("x");         //设置过滤时所需要点云类型的Z字段
    pass_x.setFilterLimits(-0.2, 0.2);      //设置在过滤字段上的范围
    pass_x.filter(*cloud_filtered);         //执行过滤，过滤结果在cloud_filtered

    pcl::PassThrough<pcl::PointXYZ> pass; //设置滤波器对象
    pass.setInputCloud(cloud_filtered);   //设置输入点云
    pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits(-1, 0.06);       //设置在过滤字段上的范围
    pass.filter(*cloud_filtered);         //执行过滤，过滤结果在cloud_filtered

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
    sor.setInputCloud(cloud_filtered);                 //设置待滤波的点云
    sor.setMeanK(50);                                  //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(1);                         //设置判断是否为离群点的阀值
    sor.filter(*cloud_filtered);

    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
    return cloud_filtered;
}


template <typename PointT>
void ProcessPointClouds<PointT>::statisticalOutlier_filter()
{
    std::cout << "检查类模板接口" << std::endl;
}




template <typename PointT>
void ProcessPointClouds<PointT>::show_pointclould(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << "显示点云" << std::endl;
    //pcl::visualization::CloudViewer viewer("pcd viewer");
   // viewer.showCloud(cloud);                                                                                                  //注释内容在类里面实现界面卡住，转化为指针viewer显示效果较好。
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    
    viewer->addPointCloud(cloud);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

template <typename PointT>
void ProcessPointClouds<PointT>::show_pointclould(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    int v1(0), v2(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);     //(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer->setBackgroundColor(0, 0, 0, v1);            //设置背景
    viewer->addText("original", 10, 10, "v1 text", v1); //设置视口名称
    viewer->addPointCloud(cloud, "sample cloud1", v1);  //添加点云

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("after", 10, 10, "v2 text", v2);
    viewer->addPointCloud(cloud_filtered, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 3, "sample cloud2");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
   