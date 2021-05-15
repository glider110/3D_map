#ifndef _BASE_H__
#define _BASE_H__
#include <pcl/common/common_headers.h>

class Object_detect
{
    public:
        virtual bool init(uint8_t) = 0;
        virtual void set_input_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr) = 0;
        virtual bool get_boundary(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&) = 0;
};





#endif
