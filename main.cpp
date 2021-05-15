#include "process/process.h"
#include "display_map/display_map.h"
#include <thread>
#include <vector>

int main()
{

    // NS_PROCESS::Process process;
    // process.init();
    // process.process();
     

    //  测试实时显示
    NS_DISPLAY_MAP::display_map dpm;
    vector<NS_DISPLAY_MAP::STR_GRIDS_FLAG> map_index_rand;

    while (1)
	{
        //随机生成数据测试
        for (size_t i = 0; i < 60; i++)
		{
            NS_DISPLAY_MAP::STR_GRIDS_FLAG  point_obtle = {rand() % 100, rand() % 100, rand() % 5};
            map_index_rand.push_back(point_obtle);
        }

        dpm.inital();
        dpm.set_input_index(map_index_rand);
        dpm.display();
        
        waitKey(1000);
	}


    system("puase");

    return 0;

}


