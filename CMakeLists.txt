cmake_minimum_required(VERSION 2.6)
project(pcl_test)

set(CMAKE_CXX_STANDARD 11)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OpenCV
find_package( OpenCV REQUIRED )  



#子模块
add_subdirectory(devices)
add_subdirectory(process)
add_subdirectory(common) 

aux_source_directory(display_map DIR_DISPALY)

set(OTHER_LIBRARIES common  devices process)
include_directories(${PROJECT_SOURCE_DIR}/devices)
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/process)
link_directories(${PROJECT_SOURCE_DIR}/build/lib)
# add_definitions(-g)

add_executable(main main.cpp ${DIR_DISPALY})
target_link_libraries (main
                ${OTHER_LIBRARIES}
                ${PCL_LIBRARIES}
                ${OpenCV_LIBS}
                pthread
                )