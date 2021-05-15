#include "muxin.h"

using namespace NS_MUXIN;

bool Muxin::init()
{
    /*******************目心相关*****************/
    const char *topic_names[] =
    {
        "stereo_depth_map",
        "mx_version",
        nullptr
    };
    connect_remote_peers(&s_remote_sub_socket, REMOTE_IP_ADDR);
    import_remote_data(topic_names, s_remote_sub_socket);
    /*******************************************/
    
    //创建NUMS_CASHE个缓冲区
    for(int i(0);i<NUMS_CASHE;i++)
    {
        cashes[i] = new int16_t[320*240];
    }
    system_on = true;

    //初始化点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr t(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud = t;

    x_min = x_max = y_min = y_max = z_min = z_max = 0;

}

void Muxin::stop()
{
    system_on = false;
    running_handle.join();
}

void Muxin::set_range_xyz(float x_min,float x_max,
float y_min,float y_max,float z_min,float z_max)
{
    this->x_min = x_min/1000.0;
    this->x_max = x_max/1000.0;
    this->y_min = y_min/1000.0;
    this->y_max = y_max/1000.0;
    this->z_min = z_min/1000.0;
    this->z_max = z_max/1000.0;
}

bool Muxin::start()
{
    running_handle = std::thread(&NS_MUXIN::Muxin::running,this);
}

void Muxin::running()
{
    printf("muxin get range image running!\n");
    char rec_topic_name[MAX_TOPIC_NAME_LEN];
    size_t option_len;
    bool not_too_close = false;
    int data_len = 0;
    msg_info_t info;
    
    uint8_t data[MAX_DDS_DATA_SIZE]={0};
    while(system_on)
    {
        data_len = zmq_recv(s_remote_sub_socket, rec_topic_name, sizeof(rec_topic_name), 0);
        if (data_len <= 0)
        {
            printf("invalid data len: %d\n", data_len);
            continue;
        }
        rec_topic_name[data_len] = '\0';
        if (!zmq_getsockopt(s_remote_sub_socket, ZMQ_RCVMORE, NULL, &option_len))
        {
            printf("invalid remote packet\n");
            continue;
        }
        data_len = zmq_recv(s_remote_sub_socket, data, sizeof(data), 0);
        memcpy(&info, data, sizeof(msg_info_t));
        if ((data_len != sizeof(msg_info_t)) || INFO_MAGIC != info.magic)
        {
            if((!strcmp(rec_topic_name, "mx_version") ) && (data_len == sizeof(mx_version_info_t)))
            {
                mx_version_info_t *mx_version = (mx_version_info_t *) (data);
                // printf("mx version %d. %d. %d. build time: %s\n", mx_version->mx_version_major,
                //        mx_version->mx_version_minor,
                //        mx_version->mx_version_patch,
                //        mx_version->build_time);
                continue;
            }
            else
            {
                printf("receve msg info failed\n");
                continue;
            }
        }
        
        if (!zmq_getsockopt(s_remote_sub_socket, ZMQ_RCVMORE, NULL, &option_len))
        {
            printf("data body not come after msg info\n");
            continue;
        }
        data_len = zmq_recv(s_remote_sub_socket, data, sizeof(data), 0);
        if (data_len != info.size)
        {
            printf("invalid data body len: %d, expect: %d\n", data_len, info.size);
            continue;
        }
        if(!strcmp(rec_topic_name, "stereo_depth_map"))
        {
            // get cali para
            camera_parameters_t cam_para;
            int32_t scale = 16;
            int32_t width = *(int32_t*)(data+12);
            int32_t height = *(int32_t*)(data+16);
            cam_para.cx = *(float*)(data+32);
            cam_para.cy = *(float*)(data+36);
            cam_para.fx = *(float*)(data+40);
            cam_para.base_line = *(float*)(data+44);
            not_too_close = *(bool*)(data+48);
            // get data pointer
            int16_t *disp = (int16_t*)(data+data_len-2*width*height);
            
            bool temp = (not_too_close == 1)?false:true;
    
            push_depth_map(disp, width, height, scale, cam_para,temp);
            
        }

    }

}

void Muxin::push_depth_map(int16_t *disp, int32_t width, 
    int32_t height, int32_t scale, camera_parameters_t camera,bool is_too_close)
{
    std::lock_guard<std::mutex> lock(mutex_cashe);
    static uint8_t index = 0;
    if(index >= NUMS_CASHE)
    {
        index = 0;
    }
    memcpy(cashes[index],disp,width*height*(sizeof(int16_t)));
    index++;
    cashe_camera = camera;
    cashe_scale = scale;
    cashe_width = width;
    cashe_height = height;
    cashe_too_close = is_too_close;
    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr& Muxin::get_depth_map(bool& too_close)
{
    std::lock_guard<std::mutex> lock(mutex_cashe);
    point_cloud->points.clear();
    int img_scale = 1;
    if( 320 == cashe_width)
    {
        img_scale = 2;
    }

    pcl::PointXYZ tmp_point;
    for (int i=0;i<cashe_height;i++)
    {
        for (int j=0;j<cashe_width;j++)
        {
            for(int k(0);k<NUMS_CASHE;k++)
            {
                float d = cashes[k][i*cashe_width+j]/(float)cashe_scale;

                if (d>0)
                {
                    tmp_point.x = (j*img_scale-cashe_camera.cx)/d*cashe_camera.base_line;
                    tmp_point.y = (i*img_scale-cashe_camera.cy)/d*cashe_camera.base_line;
                    tmp_point.z = cashe_camera.fx/d*cashe_camera.base_line;

                    if(
                        (tmp_point.x>x_min&&tmp_point.x<x_max)
                        &&(tmp_point.y>y_min&&tmp_point.y<y_max)
                        &&(tmp_point.z>z_min&&tmp_point.z<z_max)
                    )
                    {
                       
                        tmp_point.x *= 1000.0;
                        tmp_point.y *= 1000.0;
                        tmp_point.z *= 1000.0;
                        point_cloud->points.push_back(tmp_point);
                    }

                    break;      //其中一帧有就跳出循环

                }

            }

        }

    }
    too_close = cashe_too_close;
    return point_cloud;
    
}

int32_t Muxin::reproject_to_3d(int16_t *disp, int32_t width, int32_t height, int32_t scale, camera_parameters_t camera)
{
    static uint32_t count = 0;
    static uint32_t count1 = 0;
    int img_scale = 1;
    if( 320 == width)
    {
        img_scale = 2;
    }
    pcl::PointXYZ tmp_point;
    for (int i=0;i<height;i++)
        for (int j=0;j<width;j++)
        {
            count1++;
            float d = disp[i*width+j]/(float)scale;
            if (d>0)
            {
                tmp_point.x = (j*img_scale-camera.cx)/d*camera.base_line;
                tmp_point.y = (i*img_scale-camera.cy)/d*camera.base_line;
                tmp_point.z = camera.fx/d*camera.base_line;

                if(
                    (tmp_point.x>x_min&&tmp_point.x<x_max)
                    &&(tmp_point.y>y_min&&tmp_point.y<y_max)
                    &&(tmp_point.z>z_min&&tmp_point.z<z_max)
                )
                {
                    count++;
                    tmp_point.x *= 1000.0;
                    tmp_point.y *= 1000.0;
                    tmp_point.z *= 1000.0;
                    // point_cloud->points.push_back(tmp_point);
                }
                
                // printf("x=%f y=%f z=%f\n",tmp_point.x,tmp_point.y,tmp_point.z);
            }
        }
    printf("reject = %d %d\n",count,count1);
    count = 0;
    count1 = 0;
    return 0;
}   


int Muxin::connect_remote_peers(void **socket, char * ip)
{
    int ret;
    char addr[256];
    sprintf(addr, "tcp://%s:%s", ip, REMOTE_PORT);

    s_zmq_ctx = zmq_ctx_new();
    ret = zmq_ctx_set(s_zmq_ctx, ZMQ_IO_THREADS, 1);
    if (ret)
    {
        printf("zmq_ctx_set( failed, ret: %d, err: %s\n", ret, zmq_strerror(errno));
        return -1;
    }

    *socket = zmq_socket(s_zmq_ctx, ZMQ_XSUB);
    printf("connect to: %s\n", addr);
    zmq_connect(*socket, addr);
    int32_t timeout = TIME_OUT_MS;
    zmq_setsockopt(*socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    return 0;
}

int Muxin::import_remote_data(const char *names[], void *socket)
{
    char buf[MAX_TOPIC_NAME_LEN];
    buf[0] = 1;
    for (uint32_t i = 0;; i++)
    {
        if (NULL != names[i])
        {
            printf("import remote data: %s\n", names[i]);
            uint32_t topic_name_len = strlen(names[i]);
            memcpy(&buf[1], names[i], topic_name_len);
            zmq_send(socket, buf, topic_name_len + 1, 0);
        }
        else
        {
            break;
        }
    }
    return 0;
}



