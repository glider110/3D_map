#ifndef _MUXIN_H__
#define _MUXIN_H__


#ifdef __cplusplus
extern "C"{
#endif

#include <unistd.h>

#ifdef __cplusplus
}
#endif

#include <pcl/common/common_headers.h>
#include <mutex>
#include "../device.h"
#include <zmq.h>
#include <atomic>
#include <thread>

#include "mx_types.h"

#define REMOTE_IP_ADDR "10.11.11.1"
#define REMOTE_PORT "20000"
#define MAX_TOPIC_NAME_LEN (128)
#define MAX_DDS_DATA_SIZE (1024 * 1024)
#define TIME_OUT_MS 500
#define INFO_MAGIC 0x31415926
#define MAX_DESPARITY 112

namespace NS_MUXIN
{

    typedef struct
    {
        uint32_t magic;
        uint64_t reserve;
        uint32_t size;
        uint32_t index; //index
        uint64_t ts_us; //timestamp of data send
        uint32_t crc32; //crc32 val of 1st 64Byte data, POLY: 0x04C11DB7
    } msg_info_t;

    typedef struct
    {
        uint32_t width;
        uint32_t height;
        uint32_t scale;
    } depth_image_size_t;

    typedef struct mx_version_info_s
    {
        int32_t mx_version_major;
        int32_t mx_version_minor;
        int32_t mx_version_patch;
        char build_time[32];

    }mx_version_info_t;

    class Muxin:public Decives
    {
        public:
            
            virtual bool init();
            virtual bool start();
            virtual int8_t get_point_cloud(){};
            void stop();
            pcl::PointCloud<pcl::PointXYZ>::Ptr& get_depth_map(bool&);
            void set_range_xyz(float x_min,float x_max,float y_min,float y_max,float z_min,float z_max);
            
        private:
            void running();
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
            float x_min,x_max,y_min,y_max,z_min,z_max;

            void *s_zmq_ctx = nullptr;
            void *s_remote_sub_socket = nullptr;
            int connect_remote_peers(void **socket, char * ip);
            int import_remote_data(const char *names[], void *socket);
            int32_t reproject_to_3d(int16_t *disp, int32_t width, 
                int32_t height, int32_t scale
                , camera_parameters_t camera);

            std::atomic<bool> system_on;
            //缓冲区相关
            #define NUMS_CASHE 2
            int16_t* cashes[NUMS_CASHE];
            void push_depth_map(int16_t *disp, int32_t width, 
                int32_t height, int32_t scale
                , camera_parameters_t camera,
                bool is_too_close);
            
            std::mutex mutex_cashe;
            camera_parameters_t cashe_camera;
            int32_t cashe_scale;
            int32_t cashe_width;
            int32_t cashe_height;
            bool cashe_too_close;

            std::thread running_handle;
    };

}



#endif