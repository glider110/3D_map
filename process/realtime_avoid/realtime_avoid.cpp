#include "realtime_avoid.h"

bool NS_REALTIME_AVOID::Realtime_avoid::inital()
{
        vector_gridsize = vector<vector<double>>(row, vector<double>(column));
        //vector_index_merge = vector<vector<double>>(2, vector<double>(1));
      //  realtime_obstacle_detector = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        cloud_in_cliff = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
}

bool NS_REALTIME_AVOID::Realtime_avoid::set_input_imformation(/* args */)
{
        //获取障碍物接口数据
        //  realtime_obstacle_detector.get_boundary(cloud_in_obs);
        //获取悬崖数据接口数据
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cliff_cloud_vector;
        realtime_cliff_detector.get_boundary(cliff_cloud_vector);
        cloud_in_cliff = cliff_cloud_vector[0];      
}

void NS_REALTIME_AVOID::Realtime_avoid::get_env(env_type &_ahead, obstacle_location &_obs)
{
        creat_index_obstacle();
        creat_index_cliff();
        process();
        _ahead=ahead;
        _obs = obs;
}

bool NS_REALTIME_AVOID::Realtime_avoid::creat_index_obstacle(/* args */)
{
        vector_index_obstacle=realtime_obstacle_detector.get_obstacl_index();
}

bool NS_REALTIME_AVOID::Realtime_avoid::creat_index_cliff(/* args */)
{
        for (int m = 0; m < cloud_in_cliff->size(); m++) //栅格化
        {
                for (int i = -column / 2; i < column / 2; i++)
                {
                        if (((cloud_in_cliff->points[m].x > i * grid_size) && (cloud_in_cliff->points[m].x < i * grid_size + grid_size))) //如果x在某范围内
                        {
                                for (int j = 0; j < row; j++)
                                {
                                        if ((cloud_in_cliff->points[m].z > j * grid_size) && (cloud_in_cliff->points[m].z < j * grid_size + grid_size)) //如果z在某范围内
                                        {
                                                grid[i + column / 2][j].grid_inliers->indices.push_back(m);
                                                grid[i + column / 2][j].grid_inliers->header = cloud_in_cliff->header;
                                        }
                                }
                        }
                }
        }

        for (size_t i = 0; i < column; i++)
        {
                for (size_t j = 0; j < row; j++)
                {
                        {
                                vector_gridsize[i][j] = grid[i][j].grid_inliers->indices.size();
                                if ((vector_gridsize[i][j] =vector_gridsize[i][j] > 3) ? 1 :  0)
                                grid[i][j].grid_inliers->indices.clear();
                        }
                }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud_cliff(new pcl::PointCloud<pcl::PointXYZ>);
        vector<vector<int>>  vector_cliff = extrcBoundary(vector_gridsize, 1, boundary_cloud_cliff, vector_index_cliff);
}
bool NS_REALTIME_AVOID::Realtime_avoid::process(/* args */)
{
        //障碍物及悬崖索引矩阵的合并
        for (size_t i = 0; i < vector_index_obstacle[0].size(); i++)
        {
                vector_index_merge[0].insert(vector_index_merge[0].end(), vector_index_obstacle[0].begin(), vector_index_obstacle[0].end());
                vector_index_merge[1].insert(vector_index_merge[1].end(), vector_index_obstacle[1].begin(), vector_index_obstacle[1].end());
        }
        for (size_t i = 0; i < vector_index_cliff[0].size(); i++)
        {
                vector_index_merge[0].insert(vector_index_merge[0].end(), vector_index_cliff[0].begin(), vector_index_cliff[0].end());
                vector_index_merge[1].insert(vector_index_merge[1].end(), vector_index_cliff[1].begin(), vector_index_cliff[1].end());
        }

        class right
        {
        public:
                bool operator()(double val)
                {
                        return 0<=val<6;
                }
        };

        class left
        {
        public:
                bool operator()(double val)
                {
                        return 14 <= val < 20;
                }
        };

        int num_right = count_if(vector_index_merge[0].begin(), vector_index_merge[0].end(), right()); //必须要int型的数组
        int num_left = count_if(vector_index_merge[0].begin(), vector_index_merge[0].end(), left());
        vector<int> vectoer_distance=vector_index_merge[1];
        // sort(vectoer_distance.begin(), vectoer_distance.end());
        // int min_distance = vectoer_distance[0];
        vector<int>::iterator biggest = min_element(std::begin(vectoer_distance), std::end(vectoer_distance));
        int min_distance = distance(begin(vectoer_distance), biggest);
        int min_distance_x = vector_index_merge[0][min_distance];
        int min_distance_z = vector_index_merge[1][min_distance];
        obs.x=min_distance_x*20;
        obs.z = min_distance_z * 20;
        if (vector_index_merge.size() == 0) //逻辑判定是写算法特别普遍的现象，找判定因素，各个因素的相关性，if_else的次序很有讲究。
        {
                cout << "正常通过" << endl;
                env_type ahead = ENUM_NOTHING;
        }
        else
        {
                if ((num_right = 0) && (num_left=0))
                {
                        cout << "两边能通过！" << endl;
                        env_type ahead = ENUM_BOTH_SIDE;
                }
                else if(num_left = 0)
                {
                        cout << "左边能通过！" << endl;
                        env_type ahead=ENUM_LEFT_SIDE;
                }
                else if (num_right = 0)
                {
                        cout << "右边能通过！" << endl;
                        env_type ahead = ENUM_RIGHT_SIDE;
                }
                else
                {
                        cout << "不能通过！" << endl;
                        env_type ahead = ENUM_NONE_SIDE;
                }
        }  
}

