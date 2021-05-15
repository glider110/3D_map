#include "obstacle_detection.h"
#include "../p_data.h"
bool NS_OBSTACLE_DETECTION::obstacle_detection::init(uint8_t val)
{

        row = 30;
        column = 20;
        grid_size = 20;

        cloud_rct1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        cloud_rct2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        cloud_rct3 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        boundary_cloud_threshold = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        boundary_cloud_slope = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        boundary_cloud_obs = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        cloud_in_io = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        cloud_in = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));
        gridcloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new (pcl::PointCloud<pcl::PointXYZ>));

        vector_gridsize = vector<vector<double>>(column, vector<double>(row));
        vector_grid_y = vector<vector<double>>(column, vector<double>(row));
        vector_gridstd = vector<vector<double>>(column, vector<double>(row));
        vector_grid_angle = vector<vector<double>>(column, vector<double>(row));

        vector_grid_class1 = vector<vector<double>>(column, vector<double>(row));
        vector_grid_class2 = vector<vector<double>>(column, vector<double>(row));
        vector_grid_class3 = vector<vector<double>>(column, vector<double>(row));
        vector_grid_class_boundary = vector<vector<double>>(column, vector<double>(row));
}

void NS_OBSTACLE_DETECTION::obstacle_detection::pass_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        cloud_in->points.clear();
        NS_P_DATA::Data p_data = NS_P_DATA::Data::get_instence();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filter(new (pcl::PointCloud<pcl::PointXYZ>));
        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        cout << "p_data.data.ground_height-2: " << p_data.data.ground_height.max << endl;
        pass.setFilterLimits(-20, 77.7);
        //pass.setFilterLimits (-20,85);
        pass.filter(*pass_filter);

        pass.setInputCloud(pass_filter);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(100, 1000);
        pass.filter(*cloud_in);
}

void NS_OBSTACLE_DETECTION::obstacle_detection::set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        cloud_in_io = cloud;
        pass_filter(cloud_in_io);
        cout << "cloud_in: " << cloud_in->points.size() << endl;
        // cloud_in = pointCloudXYZProcessor.FilterCloud(cloud_in_io, 1, Eigen::Vector4f(-200, -20.0f, 100, 1), Eigen::Vector4f(200,p_data.data.ground_height-2, 700, 1));
}

vector<vector<double>> NS_OBSTACLE_DETECTION::obstacle_detection::get_obstacl_index()
{
        return this->indices_obs;
}

bool NS_OBSTACLE_DETECTION::obstacle_detection::get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& list)
{
        
        create_grid();
        add_grid_properity();
        merge_grid_feature();
        identify_obstacle();

        list.push_back(cloud_rct1);     //门槛
        list.push_back(cloud_rct2);     //斜坡
        list.push_back(cloud_rct3);     //障碍物
}

void NS_OBSTACLE_DETECTION::obstacle_detection::create_grid()
{
        int j = 0;
        int i = 0;
        int test_1 = 0;
        int test_2 = 0;
        int test_indices = 0;
        //grid[20][30].grid_inliers=pcl::PointIndices::Ptr(new pcl::PointIndices);
        for (int m = 0; m < cloud_in->size(); m++) //栅格化
        {
                for (i = -column / 2; i < column / 2; i++)
                {
                        if (((cloud_in->points[m].x > i * grid_size) && (cloud_in->points[m].x < i * grid_size + grid_size))) //如果x在某范围内
                        {
                                for (j = 0; j < row; j++)
                                {
                                        if ((cloud_in->points[m].z > j * grid_size) && (cloud_in->points[m].z < j * grid_size + grid_size)) //如果z在某范围内
                                        {
                                                grid[i + column / 2][j].grid_inliers->indices.push_back(m);
                                                grid[i + column / 2][j].grid_inliers->header = cloud_in->header;

                                                // grid[i + column / 2][j] .grid_cloud->push_back(t_point);
                                                //grid[i + column / 2][j].all_x += t_point.x = cloud_in->points[m].x;
                                                test_1++;
                                        }
                                        test_2++;
                                }
                        }
                }
        }

        //  printf("JJJJJJJJJJJJJJJJJJJJJJtest %d = %d  \n",test_1,test_1-cloud_in->size());
}

void NS_OBSTACLE_DETECTION::obstacle_detection::add_grid_properity()
{
        // Grid grid[column][row];
        double mean_guo = 0;
        double std_guo = 0;
        double gra_guo = 0;
        double size_guo = 0;
        int test2 = 0;
        extract.setInputCloud(cloud_in); //从哪里抽取
        for (size_t i = 0; i < column; i++)
        {
                for (size_t j = 0; j < row; j++)
                {
                        {
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clear(new pcl::PointCloud<pcl::PointXYZ>);

                                // if(
                                // (grid[i][j].grid_inliers != nullptr)
                                // &&(grid[i][j].grid_inliers->indices.size()>0)
                                // )
                                {

                                        // printf("gggggg%d \n",grid[i][j].grid_inliers->indices.size());
                                        extract.setIndices(grid[i][j].grid_inliers); //创建抽取出来的点的索引（编号）
                                        extract.setNegative(false);
                                        extract.filter(*cloud_temp); //在循环中初始化cloud_out！！！！

                                        cloud_out = cloud_temp->size() > 3 ? cloud_temp : cloud_clear;

                                        test2 += cloud_out->points.size();

                                        //printf("pppppp%d \n",cloud_out->points.size());
                                        grid[i][j].grid_cloud = cloud_out;
                                        grid[i][j].m_mean_y = get_y_mean(cloud_out);
                                        grid[i][j].m_std_y = get_y_std(cloud_out);
                                        grid[i][j].m_gradient = calAngle(cloud_out);

                                        vector_gridsize[i][j] = grid[i][j].grid_cloud->points.size();
                                        vector_grid_y[i][j] = grid[i][j].m_mean_y;
                                        vector_gridstd[i][j] = grid[i][j].m_std_y;
                                        vector_grid_angle[i][j] = grid[i][j].m_gradient;

                                        // if(grid[i][j].m_std_y > 0)
                                        // {
                                        //        test2 ++;
                                        // }
                                        size_guo += cloud_temp->size();
                                        mean_guo += grid[i][j].m_mean_y;
                                        std_guo += grid[i][j].m_std_y;
                                        gra_guo += grid[i][j].m_gradient;
                                        grid[i][j].grid_inliers->indices.clear();
                                }
                        }
                }
        }
        //printf("JJJJJJJJJJJJJJJJJJJJJJGGGGGGGGGGGggtest  = %d  \n",test2);

        // show1(vector_gridsize);
        // cout << "//////////////////////////////////////栅格内点y平均值值///////////////////////////////////////////" << endl;
        // show1(vector_grid_y);
        // cout << "//////////////////////////////////////栅格内点y标准差///////////////////////////////////////////" << endl;
        // show1(vector_gridstd);
        // cout << "//////////////////////////////////////栅格内面角度///////////////////////////////////////////" << endl;
        // show1(vector_grid_angle);
}

void NS_OBSTACLE_DETECTION::obstacle_detection::merge_grid_feature()
{
        int test2 = 0;
        for (size_t i = 0; i < column; i++)
        {
                for (size_t j = 0; j < row; j++)
                {
                        if (vector_gridstd[i][j] == 0)
                        {
                                vector_grid_class1[i][j] = 0;
                                vector_grid_class3[i][j] = 0;
                        }
                        else if (vector_gridstd[i][j] > 0 && vector_gridstd[i][j] <= 1 && vector_grid_y[i][j] >= 50.00)
                        {
                                vector_grid_class1[i][j] = 1;
                                vector_grid_class3[i][j] = 3;
                        }
                        else
                        {
                                test2++;
                                vector_grid_class1[i][j] = 3;
                                vector_grid_class3[i][j] = 3;
                        }
                }
        }
        // printf("JJJJJJJJJJJJJJJJJJJJJJGGGGGGGGGGtest = %d  \n",test2);

        int total = 0;
        for (size_t i = 0; i < vector_gridstd.size(); i++)
        {
                for (size_t j = 0; j < vector_gridstd[0].size(); j++)
                {
                        if (vector_gridstd[i][j] == 0)
                        {
                                vector_grid_class2[i][j] = 0;
                        }
                        else if (vector_gridstd[i][j] > 1 && vector_gridstd[i][j] <= 3 && vector_grid_angle[i][j] > 5 && vector_grid_angle[i][j] <= 25 && vector_grid_y[i][j] <= 70.00 && vector_grid_y[i][j] > 50.00)
                        {
                                vector_grid_class2[i][j] = 2;
                        }
                        else
                        {
                                vector_grid_class2[i][j] = 3;
                        }
                }
        }
        cout << "//////////////////////////////////////class4///////////////////////////////////////////" << endl;
        // show1(vector_grid_class1);
        // show1(vector_grid_class2);
        // show1(vector_grid_class3);
}

void NS_OBSTACLE_DETECTION::obstacle_detection::identify_obstacle()
{
        //判定障碍物逻辑    0：无数据  1：可翻越   2：斜坡   3：障碍物
       indices_threshold.clear();
       indices_slope.clear();
       indices_obs.clear(); // 障碍物
        int count_1 = statisticsValueNumbers(vector_grid_class1, 1);
        int count_2 = statisticsValueNumbers(vector_grid_class2, 2);
        cout << "cout1:" << count_1 << endl;
        cout << "cout2:" << count_2 << endl;
        if (count_1 > 20)
        {
                cout << "前方门槛" << endl;
                extrcBoundary_1(vector_grid_class1, 1, indices_threshold, indices_obs);
                add_cloud(indices_threshold, cloud_rct1);
                add_cloud(indices_obs, cloud_rct3);
        }
        else
        {
                if (count_2 > 30)
                {
                        cout << "前方斜坡" << endl;
                        extrcBoundary_1(vector_grid_class2, 2, indices_slope, indices_obs);
                        add_cloud(indices_slope, cloud_rct2);
                        add_cloud(indices_obs, cloud_rct3);
                }
                else
                {
                        cout << "前方障碍物" << endl;
                        vector<vector<int>> vector3 = extrcBoundary(vector_grid_class3, 3, boundary_cloud_obs, indices_obs);
                        add_cloud(indices_obs, cloud_rct3);
                }
        }
}

void NS_OBSTACLE_DETECTION::obstacle_detection::add_cloud(vector<vector<double>> myvector, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_rct)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp11(new pcl::PointCloud<pcl::PointXYZ>);
        int a = 0;
        int count = 0;
        //printf("ggggggsize %d \n",myvector[0].size());
        for (size_t k = 0; k < myvector[0].size(); k++)
        {

                if (myvector[1][k] != 30) //筛选出没有数据的点

                {

                        int ii = myvector[0][k];
                        int jj = myvector[1][k];
                        cloud_temp = grid[ii][jj].grid_cloud;
                        *cloud_rct = *cloud_rct + *cloud_temp;
                        count++;
                        printf(" %d ", cloud_temp->points.size());
                }
        }
        // while(1)sleep(1);
        // printf("gggggg %d \n", count);
        // printf("\n\n\n\n\ngggggg %d \n", count);
        printf("\n\n\n\n");
        //   printf("ggggggggggg %d \n",cloud_rct->points.size());
}

