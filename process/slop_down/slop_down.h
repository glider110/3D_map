#ifndef _SLOP_DOWN_H__
#define _SLOP_DOWN_H__
#include <pcl/common/common_headers.h>
#include "../p_data.h"
#include "../base.h"
namespace NS_SLOP_DOWN
{

    class Slop_down:public Object_detect
    {
        public:
            bool init(uint8_t);
            void set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr);
            bool get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);
        private:
            NS_P_DATA::Data p_data;
            pcl::PointCloud<pcl::PointXYZ>::Ptr original_points;

            void process();

            pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points;
            int threshold_ground_down = 100;        //  mm

    };

}


#endif