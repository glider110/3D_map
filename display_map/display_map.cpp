#include "display_map.h"

void NS_DISPLAY_MAP::display_map::inital()
{
        row_map = 100;
        col_map = 100;
        this->Img = cv::Mat(row_map, col_map, CV_8UC3, Scalar(255, 255, 255));
}

void NS_DISPLAY_MAP::display_map::set_input_index(const std::vector<STR_GRIDS_FLAG> myindex)
{
        map_index = myindex;
}

void NS_DISPLAY_MAP::display_map::display()
{
        vector<vector<int>> color_matrix;
        vector<int> color_red = {255, 0, 0};
        vector<int> color_green = {0, 255, 0};
        vector<int> color_blue = {0, 0, 255};
        vector<int> color_cyan = {255, 255, 0};
        vector<int> color_yellow = {0, 255, 255};
        color_matrix.push_back(color_red);
        color_matrix.push_back(color_green);
        color_matrix.push_back(color_blue);
        color_matrix.push_back(color_cyan);
        color_matrix.push_back(color_yellow);

        for (size_t i = 0; i < map_index.size(); i++)
        {
                for (size_t j = 0; j < 3; j++)
                {
                        Img.at<cv::Vec3b>(map_index[i].x_index, map_index[i].y_index)[j] = color_matrix[map_index[i].flag][j];
                }
        }
        namedWindow("GRID_SHOW", WINDOW_NORMAL); //构建一个窗口，自适应图片大小
        resizeWindow("GRID_SHOW", 600, 600);
        imshow("GRID_SHOW", Img);
}
