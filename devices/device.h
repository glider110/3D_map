#ifndef _DEVICE_H__
#define _DEVICE_H__


class Decives
{
    public:
        virtual bool init() = 0;
        virtual bool start() = 0;
        virtual int8_t get_point_cloud() = 0;
        virtual ~Decives(){};
};


#endif
