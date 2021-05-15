#include "../obstacle_detection/obstacle_detection.h"
#include "../cliff_detection/cliff_detection.h"
#include <algorithm> //vector find_if
#define column 30
#define row 20
#define grid_size 20;
/**
 * 算法说明：实时避障只关注悬崖和前方障碍物情况
 * 算法步骤：
 * 1.接入障碍物检测接口，获取索引位置。
 * 2.接入悬崖检测接口，得到悬崖点云，映射到栅格，获取索引位置。
 * 3.融合上述两个矩阵。
 * 4.逻辑判定避障类型。
*/

namespace  NS_REALTIME_AVOID
{
          struct obstacle_location
    {
        float x;
        float z;
    };

    enum env_type
    {
            //障碍物类型
            ENUM_NOTHING,
            ENUM_BOTH_SIDE,
            ENUM_LEFT_SIDE,
            ENUM_RIGHT_SIDE,
            ENUM_NONE_SIDE,
            //too close
            //  ENUM_TOO_CLOSE,
    };

    class Realtime_avoid
    {
    public:
            bool inital();
            bool set_input_imformation();
            void get_env(env_type &_ahead, obstacle_location &_obs);

    private:
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_cliff;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_obs;
            vector<vector<double>> vector_index_obstacle;
            vector<vector<double>> vector_index_cliff;
            vector<vector<int>> vector_index_merge;
            vector<vector<double>> vector_gridsize;
            NS_OBSTACLE_DETECTION::obstacle_detection realtime_obstacle_detector;
            NS_CLIFF_DECTECTION::Cliff_detection realtime_cliff_detector;
            bool creat_index_obstacle();
            bool creat_index_cliff();
            bool process();
            Grid grid[20][30];
            env_type ahead;
            obstacle_location obs;
        };
        
     
        
} // namespace  NS_REALTIME_AVOID
