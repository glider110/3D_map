#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <vector>

//#include "../process/process.h"
using namespace cv;
using namespace std;

namespace NS_DISPLAY_MAP
{
        struct STR_GRIDS_FLAG //地图索引输出，带属性
        {
                int x_index;
                int y_index;
                uint8_t flag;
        };

        class display_map
        {
        public:
                void inital();
                void set_input_index(const std::vector<STR_GRIDS_FLAG> myindex);
                void display();

        private:
                int row_map;
                int col_map;
                Mat Img;
                std::vector<STR_GRIDS_FLAG> map_index;
        };

} // namespace NS_DISPLAY_MAP
