#include "rtime.h"

// #include <string.h>
// #include <iostream>
namespace NS_TIMEMGR
{
    void TimeMgr::time_init()
    {
        struct timespec t;
        clock_gettime( CLOCK_MONOTONIC, &t );
        int64_tt timeNanoSecRealTime = t.tv_sec * 1000000000LL + t.tv_nsec;
        begin_timeUs = timeNanoSecRealTime/1000LL;
    }

    int64_tt TimeMgr::time_duaUs()
    {
        struct timespec t;
        clock_gettime( CLOCK_MONOTONIC, &t );
        int64_tt end = t.tv_sec * 1000000000LL + t.tv_nsec;
        taked_timeUs = end/1000LL - begin_timeUs;
        return taked_timeUs;
    }

    int64_tt TimeMgr::time_duaS()
    {

        auto get_local_time = [](char *time_str, int len, struct timeval *tv)->char *
        {
            struct tm* ptm;
            char time_string[40];
            long milliseconds;
            
            ptm = localtime (&(tv->tv_sec));
        
            /* 格式化日期和时间，精确到秒为单位。*/
            strftime (time_string, sizeof(time_string), "%Y\\%m\\%d %H-%M-%S", ptm); //输出格式为: 2018\12\09 10-52-28.302
        
            /* 从微秒计算毫秒。*/
            milliseconds = tv->tv_usec / 1000;
        
            /* 以秒为单位打印格式化后的时间日期，小数点后为毫秒。*/
            // snprintf (time_str, len, "%s.%03ld", time_string, milliseconds);
        
            return time_str;
        };
        return time_duaUs()/1000000LL;

    }

    int64_tt TimeMgr::time_eraUs()
    {
        int64_tt time;
        time = time_duaUs();
        time_init();
        return time;
    }
}