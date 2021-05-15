#include "process.h"
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include "time/rtime.h"
#include <algorithm>
#include "cliff_detection/cliff_detection.h"
#include "slop_down/slop_down.h"
#include "pass_through/pass_through.h"
#include "obstacle_detection/obstacle_detection.h"
#define INDEX(x)    get_obj_index(x)

namespace NS_PROCESS
{
    void Process::init()
    {
        //目心设备初始化，并设置初步截取范围
        float x_min,x_max,y_min,y_max,z_min,z_max;
        x_min = -1000;
        x_max = 1000;
        y_min = -800;
        y_max = 100;
        z_min = 0;
        z_max = 1000;
        muxin.init();
        muxin.set_range_xyz(x_min,x_max,-215,100,100,1000);

        
        //本模块变量初始化
        pre_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        original_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        is_corection = false;
        system_on = true;

        //初始化障碍物类型及等级
        set_object_range(ENUM_SLOP_DOWN,0);
        set_object_range(ENUM_CLIFF,1);
        set_object_range(ENUM_SLOP_UP,2);
        set_object_range(ENUM_THRESHOULD,3);
        set_object_range(ENUM_GENERAL,4);
        set_object_range(ENUM_PASSTHROUGH,5);
        
        //初始化点云指针索引
        int obj_nums = range_sort();
        for(int i(0);i < obj_nums;i++)
        {
            object_ptr.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
        }

        //各种障碍物检测模块初始化
        pCliffHandle = std::shared_ptr<Object_detect>(new NS_CLIFF_DECTECTION::Cliff_detection);
        pSlopDownHandle = std::shared_ptr<Object_detect>(new NS_SLOP_DOWN::Slop_down);
        pPassThroughHandle = std::shared_ptr<Object_detect>(new NS_PASS_THROUGH::Pass_through);
        pObstacleHandle = std::shared_ptr<Object_detect>(new NS_OBSTACLE_DETECTION::obstacle_detection);
        
        pCliffHandle->init(NULL);
        pSlopDownHandle->init(NULL);
        pPassThroughHandle->init(NULL);
        pObstacleHandle->init(NULL);

        pre_treatment.init();

    }

    void Process::process()
    {
        //启动传感器设备
        muxin.start();

        //可视化相关
        int frame = 0;
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
        
        while(system_on)
        {
            /*******************************
             * 1、获取设备点云
            *******************************/
            bool is_too_close = false;
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = muxin.get_depth_map(is_too_close);
            if(point_cloud->points.size() <= 0)//初始化时点云可能为0
            {
                usleep(100*1000);
                continue;        
            }
            NS_TIMEMGR::TimeMgr time;
            time.time_init();
            pcl::copyPointCloud(*point_cloud, *original_points);
            

            /*******************************
             * 2、点云校正
             * 目前只会运行一次
            *******************************/
            if(!is_corection)
            {
                printf("original_points size = %d\n",(int)original_points->points.size());
                pre_treatment.calibration(original_points);
                is_corection = true;
                continue;
            }


            /*******************************
             * 3、点云预处理
            *******************************/
            pre_treatment.get_points(original_points,pre_points);
            // printf("delay %lld ms\n",time.time_eraUs()/1000);


            /*******************************
             * 4、使用线程池对任务进行分配
             *******************************/

            //任务打包
            auto thread_cliff_handle = [&]()
            {
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boundary_points;
                pCliffHandle->set_input_points(pre_points);
                bool ret = pCliffHandle->get_boundary(boundary_points);
                object_ptr[INDEX(ENUM_CLIFF)] = boundary_points[0];
            };

            auto thread_slop_down_handle = [&]()
            {
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boundary_points;
                pSlopDownHandle->set_input_points(pre_points);
                bool ret = pSlopDownHandle->get_boundary(boundary_points);
                object_ptr[INDEX(ENUM_SLOP_DOWN)] = boundary_points[0];
            };

            auto thread_pass_through_handle = [&]()
            {
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boundary_points;
                pPassThroughHandle->set_input_points(pre_points);
                bool ret = pPassThroughHandle->get_boundary(boundary_points);
                object_ptr[INDEX(ENUM_PASSTHROUGH)] = boundary_points[0];
            };

            auto thread_obstacle_detection_handle = [&]()
            {
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boundary_points;
                pObstacleHandle->set_input_points(pre_points);
                bool ret = pObstacleHandle->get_boundary(boundary_points);
                object_ptr[INDEX(ENUM_THRESHOULD)] = boundary_points[0];
                object_ptr[INDEX(ENUM_SLOP_UP)] = boundary_points[1];
                object_ptr[INDEX(ENUM_GENERAL)] = boundary_points[2];
            };


            //线程处理
            thread_pools.add_task(thread_cliff_handle);
            thread_pools.add_task(thread_slop_down_handle);
            thread_pools.add_task(thread_pass_through_handle);
            thread_pools.add_task(thread_obstacle_detection_handle);
            
            //等待所有任务工作完毕
            while(thread_pools.is_working())    
            {
                usleep(1*1000);
            }

            //获取当前位姿
            STR_ATTITUDE position_t;

            /*******************************
             * ４、坐标转换、边缘信息整合、地图映射
            *******************************/
            mapping_grid(position_t);


            //显示
            auto display3 = [&]()
            {

                char buf1[233],buf2[233],buf3[233],buf4[233];
                if (frame != 0)
                {
                    sprintf(buf1,"frame%d",frame);
                    sprintf(buf2,"frame%d",frame);
                    sprintf(buf3,"frame%d",frame);
                    sprintf(buf4,"frame%d",frame);
                    // viewer->removePointCloud(buf1);
                }
                sprintf(buf1,"frame%d", ++frame);
                sprintf(buf2,"frame%d", ++frame);
                sprintf(buf3,"frame%d", ++frame);
                sprintf(buf4,"frame%d", ++frame);

                int v1 = 0;
                int v2 = 0;
                // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
                //原始点云窗口
                viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
                // viewer->setBackgroundColor(0, 0, 0, v1);
                // viewer->addText("original", 10, 10, "v1 text", v1);
                viewer->addPointCloud<pcl::PointXYZ>(pre_points, buf1, v1);
                // viewer->addCoordinateSystem(1.0);
                // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
                //校正窗口
                viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
                // viewer->setBackgroundColor(0, 0, 0, v2);
                // viewer->addText("horizontal", 10, 10, "v2 text", v2);
                viewer->addPointCloud<pcl::PointXYZ>(object_ptr[INDEX(ENUM_CLIFF)], buf2, v2);
                // viewer->addCoordinateSystem(1.0);
                // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
                viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v2);
                viewer->addPointCloud<pcl::PointXYZ>(object_ptr[INDEX(ENUM_SLOP_DOWN)], buf3, v2);
                
                viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2);
                viewer->addPointCloud<pcl::PointXYZ>(object_ptr[INDEX(ENUM_PASSTHROUGH)], buf4, v2);

                viewer->spinOnce(100);  //刷新
            };
            
            auto display2 = [&]()
            {
                
                char buf[233];
                if (frame != 0)
                {
                    sprintf(buf,"frame%d",frame);
                    viewer->removePointCloud(buf);
                }
                sprintf(buf,"frame%d", ++frame);
                // viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
                viewer->addPointCloud<pcl::PointXYZ> (pre_points,buf);
                viewer->spinOnce ();
                // printf("sisze = %d\n",cliff_points->points.size());
            };
            // display2();
            display3();


        }
    }


    void Process::coordinate_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr& input
    ,STR_ATTITUDE xy_angle)
    {
        /**
         * 当前3D模组坐标系为：右－x,下－y,前－ｚ
         * 机器坐标系为:前－ｘ，左－y
         * 角度为左加右减
        */

        if(input->points.size() < 1)return;
        float angle = 0;
        Eigen::Vector3f p_R = {0,-1,0};
        
    #if 0       //-180~180，左加右减
        if(xy_angle.angle > 0)
        {
            angle = xy_angle.angle*M_PI/180.0;
        }
        else
        {
            p_R = -1 * p_R;
            angle = -1*xy_angle.angle*M_PI/180.0;
        }

    #else       //0~360，左加右减
        angle = xy_angle.angle*M_PI/180.0;

    #endif

        //坐标转换
        Eigen::Matrix4f rotateMatrix = Eigen::Matrix4f::Identity();
        rotateMatrix(0, 0) = cos(angle) + p_R[0] * p_R[0] * (1 - cos(angle));
        rotateMatrix(0, 1) = p_R[0] * p_R[1] * (1 - cos(angle) - p_R[2] * sin(angle));
        rotateMatrix(0, 2) = p_R[1] * sin(angle) + p_R[0] * p_R[2] * (1 - cos(angle));

        rotateMatrix(1, 0) = p_R[2] * sin(angle) + p_R[0] * p_R[1] * (1 - cos(angle));
        rotateMatrix(1, 1) = cos(angle) + p_R[1] * p_R[1] * (1 - cos(angle));
        rotateMatrix(1, 2) = -p_R[0] * sin(angle) + p_R[1] * p_R[2] * (1 - cos(angle));

        rotateMatrix(2, 0) = -p_R[1] * sin(angle) + p_R[0] * p_R[2] * (1 - cos(angle));
        rotateMatrix(2, 1) = p_R[0] * sin(angle) + p_R[1] * p_R[2] * (1 - cos(angle));
        rotateMatrix(2, 2) = cos(angle) + p_R[2] * p_R[2] * (1 - cos(angle));

        rotateMatrix(0, 3) = -1*xy_angle.y - y_bias;
        rotateMatrix(1, 3) = 0;
        rotateMatrix(2, 3) = xy_angle.x + x_bias;

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*input, *input, rotateMatrix);

        
    }

    void Process::mapping_grid(STR_ATTITUDE position_t)
    {
        /**
         * 障碍物类型及优先级：数值小对应高级别
         * 下斜坡－0
         * 悬崖－1
         * 上斜坡－2
         * 门槛－3
         * 普通障碍物－4
         * 沙发等低矮物－5
         * 
         * 注意：
         * １、下采样对输出点作尺度归一化
         * ２、高优先级直接替换低优先级，设定满足点数量阈值
        */

       
       pcl::VoxelGrid<pcl::PointXYZ> sor;
       float voxel = 10;

       for(int i(0);i < object_ptr.size();i++)
       {
            //尺度大小归一化
            sor.setInputCloud(object_ptr[i]);
            sor.setLeafSize (voxel, voxel, voxel);
            sor.filter (*object_ptr[i]);

            //坐标系转换
            coordinate_transform(object_ptr[i],position_t);

            //点云到地图映射，并记数
            for(int j(0);j<object_ptr[i]->points.size();j++)
            {
                STR_GRIDS_FLAG_COUNT temp(object_ptr.size());
                temp.x_index = object_ptr[i]->points[j].z / 50;
                temp.y_index = (-1*object_ptr[i]->points[j].x) / 50;

                std::vector<STR_GRIDS_FLAG_COUNT>::iterator iter;
                iter = find(grids_count.begin(), grids_count.end(), temp);
                if(iter != grids_count.end())
                {
                    iter->count[i]++;
                }
                else
                {
                    temp.count[i]++;
                    grids_count.push_back(temp);
                }
            }
       }


       //地图栅格属性确定，生成地图索引
       for(int i(0);i < grids_count.size();i++)
       {
            for(int j(0);j < object_ptr.size();j++)
            {
                if(grids_count[i].count[j] >= 2)
                {
                    STR_GRIDS_FLAG temp;
                    temp.x_index = grids_count[i].x_index;
                    temp.y_index = grids_count[i].y_index;
                    temp.flag = j;
                    map_index.push_back(temp);
                    break;
                }
            }

       }

    }


    bool Process::set_object_range(ENUM_OBJECT_CLASS class_object,int range)
    {
        if((map_range_obj.find(range) != map_range_obj.end())&&(map_range_obj.size() != 0))
        {
            printf("已存在该等级\n");
            return false;
        }
        map_range_obj.insert(std::pair<int,ENUM_OBJECT_CLASS>(range, class_object));
        
    }

    int Process::range_sort()
    {
        int jj = 0;
        for(auto i = map_range_obj.begin();i != map_range_obj.end();i++)
        {
            ENUM_OBJECT_CLASS t_find_state = i->second;
            // printf("hhh %d  %d\n",t_find_state,jj);
            map_obj_range.insert(std::pair<ENUM_OBJECT_CLASS,int>(t_find_state, jj));
            jj++;
        }
        return jj;
    }

    int Process::get_obj_index(ENUM_OBJECT_CLASS obj)
    {
        auto i = map_obj_range.find(obj);

        if(i == map_obj_range.end())
        {
            printf("不存在此种障碍物类型\n");
            return -1;
        }
        return i->second;

    }

}

