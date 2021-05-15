#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

// #include <sys/time.h>
#include <time.h>
#ifdef __cplusplus
}
#endif

// typedef unsigned char       uint8_t;
// typedef unsigned short      uint16_t;
// typedef unsigned int        uint32_t;
// typedef unsigned long long  uint64_t;

typedef signed char         int8_t;
typedef signed short        int16_t;
typedef signed int          int32_t;
typedef signed long long    int64_tt;

namespace NS_TIMEMGR
{
    class TimeMgr
    {
        public:
            TimeMgr():begin_timeUs(0),taked_timeUs(0)
            {
                time_init();
            };
            ~TimeMgr(){};
            void time_init();
            int64_tt time_duaUs();
            int64_tt time_eraUs();
            int64_tt time_duaS();
        private:
            int64_tt begin_timeUs;
            int64_tt taked_timeUs;

    };

}



