#include "customWidget.h"
#define COL 20
#define ROW 30

void show(vector<vector<int>> &v)
{
        for (int i = 0; i < (int)v.size(); i++)
        {
                for (int j = 0; j < (int)v[0].size(); j++)
                {
                        cout << v[i][j] <<"    ";
                }
                cout << endl;
        }
        cout << endl;
}

void show1(vector<vector<double>> &v)
{
        for (int i = 0; i < (int)v.size(); i++)
        {
                for (int j = 0; j < (int)v[0].size(); j++)
                {
                        cout << v[i][j] << "    ";
                }
                cout << endl;
                cout << endl;
        }
        cout << endl;
        cout << endl;
}

void show1_chw(vector<vector<double>> &v,int& total)
{
        for (int i = 0; i < (int)v.size(); i++)
        {
                for (int j = 0; j < (int)v[0].size(); j++)
                {
                        // cout << v[i][j] << "    ";
                        if(v[i][j] == 3)
                        {
                                total += j;
                        }
                }
                cout << endl;
                cout << endl;
        }
        cout << endl;
        cout << endl;
}

void show2(vector<vector<int>> &v)
{
        for (int i = 0; i < (int)v.size(); i++)
        {
                for (int j = 0; j < (int)v[0].size(); j++)
                {
                        cout << v[i][j] << "    ";
                }
                cout << endl;
        }
        cout << endl;
}

void extrcBoundary_1(vector<vector<double>> vectorBoundary, int type_value, vector<vector<double>> &boudary_merge1, vector<vector<double>> &boudary_merge3)
{
        vector<double> boudary_z_1;
        vector<double> boudary_x_1;
        vector<double> boudary_z_3;
        vector<double> boudary_x_3;
        vector<int> inlier_vec;
        for (size_t i = 0; i < COL; i++)
         {
                        vector<double>::iterator it1 = find(vectorBoundary[i].begin(), vectorBoundary[i].end(), type_value);
                        int index1 = std::distance(std::begin(vectorBoundary[i]), it1);
                        vector<double>::iterator it2 = find(vectorBoundary[i].begin(), vectorBoundary[i].end(), 3);
                        int index3 = std::distance(std::begin(vectorBoundary[i]), it2);

                        if ((index1 != 30 )&& (index3 != 30))
                        {
                                int temp=index1>index3?index3:index1;
                                 boudary_x_1.push_back(i);
                                 boudary_z_1.push_back(temp); //找到第一个1的位置
                         }
                        else if ((index3==30)&&(index1!= 30 ))
                        {
                                 boudary_x_1.push_back(i);
                                 boudary_z_1.push_back(index1); //找到第一个1的位置
                        }
                        else if ((index1==30)&&(index3!= 30 ))
                        {
                                boudary_x_3.push_back(i);
                                boudary_z_3.push_back(index3);
                        }
                       else
                        {
                                        // boudary_x_3.push_back(i);
                                        // boudary_z_3.push_back(index3);
                        }
                       
         }   
         boudary_merge1.push_back(boudary_x_1);
         boudary_merge1.push_back(boudary_z_1);
        boudary_merge3.push_back(boudary_x_3);
        boudary_merge3.push_back(boudary_z_3); 
        // show1(boudary_merge1);
         //show1(boudary_merge3);     


}

vector<vector<int>> extrcBoundary(vector<vector<double>> vectorBoundary, int value, pcl::PointCloud<pcl::PointXYZ>::Ptr &boundary_cloud, vector<vector<double>> &boudary_merge)
{
        vector<double> boudary_z;
        vector<double> boudary_x;
         vector<double> boudary_size;
        //vector<vector<double>> m_boudary_merge;
        vector<int> inlier_vec;

                for (size_t i = 0; i < COL; i++)
                {
                        vector<double>::iterator it = find(vectorBoundary[i].begin(), vectorBoundary[i].end(), value);
                        int index1 = std::distance(std::begin(vectorBoundary[i]), it);
                        //cout << "find:" << index1 << endl;
                        boudary_x.push_back(i);
                        boudary_z.push_back(index1);
                        index1=0;
                }
                vector<vector<int>> boudary_matrix(COL, vector<int>(ROW));

                // boudary_merge.clear();
               boudary_merge.push_back(boudary_x);
                boudary_merge.push_back(boudary_z);
                // printf("\n");
                // cout << endl;
                // show1(boudary_merge);
                // printf("\n");
                // cout << endl;
                
        // }
        // printf("________________________________________\n");

        for (size_t k = 0; k < boudary_merge[0].size(); k++)
        {
                if (boudary_merge[1][k]!=ROW)//筛选出没有数据的点
                {
                        int ii = boudary_merge[0][k];
                        int jj = boudary_merge[1][k];
                        //点云显示
                        pcl::PointXYZ point_grid;

                        point_grid.x = ii;
                        point_grid.y=jj;
                        point_grid.z=0;
                        boundary_cloud->points.push_back(point_grid);
                        boudary_matrix[ii][jj] = 1;      

                }
        }
        return boudary_matrix;
}

vector<vector<int>> extrcBoundary_obs(vector<vector<double>> vectorBoundary, int value1,int value2, pcl::PointCloud<pcl::PointXYZ>::Ptr &boundary_cloud, vector<vector<double>> &boudary_merge)
{
        vector<double> boudary_z;
        vector<double> boudary_x;
        //vector<vector<double>> m_boudary_merge;
        vector<int> inlier_vec;

        for (size_t i = 0; i < COL; i++)
        {
                vector<double>::iterator it1 = find(vectorBoundary[i].begin(), vectorBoundary[i].end(), value1);
                int index1 = std::distance(std::begin(vectorBoundary[i]), it1);
                vector<double>::iterator it2 = find(vectorBoundary[i].begin(), vectorBoundary[i].end(), value2);
                int index2 = std::distance(std::begin(vectorBoundary[i]), it2);
                if (index1==30)
                {
                        boudary_x.push_back(i);
                        boudary_z.push_back(index2);
             }
            
        }
        vector<vector<int>> boudary_matrix(COL, vector<int>(ROW));

        // boudary_merge.clear();
        boudary_merge.push_back(boudary_x);
        boudary_merge.push_back(boudary_z);
        // printf("\n");
        // cout << endl;
        show1(boudary_merge);
        // printf("\n");
        // cout << endl;

        // }
        // printf("________________________________________\n");

        for (size_t k = 0; k < boudary_merge[0].size(); k++)
        {
                if (boudary_merge[1][k] != ROW) //筛选出没有数据的点
                {
                        int ii = boudary_merge[0][k];
                        int jj = boudary_merge[1][k];
                        //点云显示
                        pcl::PointXYZ point_grid;

                        point_grid.x = ii;
                        point_grid.y = jj;
                        point_grid.z = 0;
                        boundary_cloud->points.push_back(point_grid);
                        boudary_matrix[ii][jj] = 1;
                }
        }
        return boudary_matrix;
}

int statisticsValueNumbers(vector<vector<double>> vectorBoundary, int value)
{
        int temp = 0;
        for (size_t i = 0; i < COL; i++)
        {
                int num = count(vectorBoundary[i].begin(), vectorBoundary[i].end(), value);
                // int num = 4;
                // cout << "cout:" << num << endl;
                temp += num;
         }

        return temp;
}

    void savedata(vector<vector<double>> &v)
{
        ofstream ofs;
        ofs.open("data.txt", ios::out) ;
        for (int i = 0; i < (int)v.size(); i++)
        {
                for (int j = 0; j < (int)v[0].size(); j++)
                {
                        ofs << v[i][j] << "\t\t";
                        cout << v[i][j] << "\t";
                }
                cout << endl;
                ofs << endl;
        }
        cout << endl;
        ofs << endl;
        ofs.close();
}

double calAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        if (cloud->size()==0)
        {
                double dAngle = -2;
                return 0;
        }
        else
        {
                ProcessPointClouds<pcl::PointXYZ> pointCloudXYZProcessor_grid;
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                plane_cloud = pointCloudXYZProcessor_grid.mysegmentplane(cloud, coefficients);
                if (plane_cloud->size() > 3)
                {
                        myVect3 incline_norm, floor_norm;
                        double dAngle = 0;
                        get_norm(coefficients, floor_norm, incline_norm);
                        get_Angle(incline_norm, floor_norm, dAngle);
                        return dAngle;
                }
                else
                {
                        double dAngle = -1;
                        return 0;
                } 
                
        }
        
        
}

float_t get_y_mean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        float_t temp = 0;
        vector<float_t> temp_vector;
        for (size_t i = 0; i < cloud->size(); i++)
        {
                temp_vector.push_back(cloud->points[i].y);
        }
        if (cloud->size()>0)
        {
                double sum = std::accumulate(std::begin(temp_vector), std::end(temp_vector), 0.0);
                double mean = sum / temp_vector.size(); //均值
                return mean;
        }
       else
       {
              return 0;
       }
       
}

float_t get_y_std(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

        float_t temp = 0;
        vector<float_t> temp_vector;
        for (size_t i = 0; i < cloud->size(); i++)
        {
                temp_vector.push_back(cloud->points[i].y);
        }
        double sum = std::accumulate(std::begin(temp_vector), std::end(temp_vector), 0.0);
        double mean = sum / temp_vector.size(); //均值
        double accum = 0.0;
        std::for_each(std::begin(temp_vector), std::end(temp_vector), [&](const double d) {
                accum += (d - mean) * (d - mean);
        });
        double stdev = sqrt(accum / (temp_vector.size() - 1)); //方差
        return stdev;
}

bool get_Angle(myVect3 v1, myVect3 v2, double &angle_abs)
{
        double a1 = v1.x;
        double b1 = v1.y;
        double c1 = v1.z;
        double a2 = v2.x;
        double b2 = v2.y;
        double c2 = v2.z;
        double k;

        k = (a1 * a2 + b1 * b2 + c1 * c2) / sqrt(a1 * a1 + b1 * b1 + c1 * c1) / sqrt(a2 * a2 + b2 * b2 + c2 * c2);
        double dAngle = acos(k);
        double angle = (dAngle / PI) * 180;
        // cout << "angle:" << angle << endl;
        if (angle >= 90)
        {
                angle_abs = 180 - angle;
        }
        else
        {
                angle_abs = angle;
        }
      //  cout << "dAngle:" << dAngle << endl;
       // cout << "angle_abs:" << angle_abs << endl;
        return 0;
}

void get_norm(pcl::ModelCoefficients::Ptr coefficients, myVect3 &floor_norm, myVect3 &incline_norm)
{
        // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        //           << coefficients->values[1] << " "
        //           << coefficients->values[2] << " "
        //           << coefficients->values[3] << std::endl;

        floor_norm.x = 0;
        floor_norm.y = 1;
        floor_norm.z = 0;
        incline_norm.x = coefficients->values[0];
        incline_norm.y = coefficients->values[1];
        incline_norm.z = coefficients->values[2];

        //std::cerr << "结构体qqqqq" << floor_norm.y << std::endl;
}
