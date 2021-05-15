#ifndef _PROCESS_H__
#define _PROCESS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#include "muxin/muxin.h"
#include "thread_pool/thread_pool.h"
#include "pretreatment/pretreatment.h"
#include <atomic>
#include <pcl/visualization/cloud_viewer.h>
#include "base.h"
#include <map>
#include <vector>

namespace NS_PROCESS
{
    enum ENUM_OBJECT_CLASS:uint8_t
    {
        ENUM_CLIFF,
        ENUM_SLOP_DOWN,
        ENUM_SLOP_UP,
        ENUM_THRESHOULD,
        ENUM_GENERAL,
        ENUM_PASSTHROUGH,
    };

    struct STR_ATTITUDE     //机器姿态信息
    {
        float x;
        float y;
        float angle;
    };

    struct STR_GRIDS_FLAG     //地图索引输出，带属性
    {
        int x_index = 0;
        int y_index = 0;
        uint8_t flag;
    };

    struct STR_GRIDS_FLAG_COUNT
    {
        STR_GRIDS_FLAG_COUNT(int nums)
        {
            count = new uint32_t [nums];
            
        }
        ~STR_GRIDS_FLAG_COUNT()
        {
            if(count != nullptr)
            {
                delete [] count;
            }
        }
        
        bool operator == (const STR_GRIDS_FLAG_COUNT & obj) const
        {
            return x_index == obj.x_index && y_index == obj.y_index;
        }

        int x_index;
        int y_index;
        uint32_t* count;

    };

    class Process
    {
        public:
            void init();
            void process();
        private:
            NS_MUXIN::Muxin muxin;
            NS_THREAD_POOL::Thread_pool thread_pools;
            pcl::PointCloud<pcl::PointXYZ>::Ptr original_points;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pre_points;

            NS_PRETREATMENT::Pretreatment pre_treatment;
            std::shared_ptr<Object_detect> pCliffHandle;
            std::shared_ptr<Object_detect> pSlopDownHandle;
            std::shared_ptr<Object_detect> pPassThroughHandle;
            std::shared_ptr<Object_detect> pObstacleHandle;

            bool is_corection = false;
            std::atomic<bool> system_on;

            //坐标转换相关
            void coordinate_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr&
            ,STR_ATTITUDE);
            float y_bias = 35;          //３d传感器和机器中心相对位置
            float x_bias = 185;

            //地图映射
            void mapping_grid(STR_ATTITUDE);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_ptr;
            std::vector<STR_GRIDS_FLAG> map_index;  //存放最终输出的索引＋标志
            std::vector<STR_GRIDS_FLAG_COUNT> grids_count;  //点云映射记数

            //map设置等级相关
            using RANGE_OBJECT_MAP = std::map<int,ENUM_OBJECT_CLASS,std::less<int>>;
            using OBJECT_RANGE_MAP = std::map<ENUM_OBJECT_CLASS,int >;
            RANGE_OBJECT_MAP map_range_obj;
            OBJECT_RANGE_MAP map_obj_range;
            bool set_object_range(ENUM_OBJECT_CLASS class_object,int range);
            int range_sort();
            int get_obj_index(ENUM_OBJECT_CLASS);

            //可视化相关
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    };

}


#endif
