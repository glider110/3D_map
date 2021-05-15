#ifndef _PASS_THROUGH_H__
#define _PASS_THROUGH_H__
#include <pcl/common/common_headers.h>
#include "../p_data.h"
#include "../base.h"
namespace NS_PASS_THROUGH
{

    class Pass_through:public Object_detect
    {
        public:
            bool init(uint8_t);
            void set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr);
            bool get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&);

        private:
            NS_P_DATA::Data p_data;
            pcl::PointCloud<pcl::PointXYZ>::Ptr original_points;

            pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points;
            int threshold_dt_height = 40;        //超出机身最高高度mm
    };

}


#endif