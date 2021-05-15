cmake_minimum_required(VERSION 2.6)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


aux_source_directory(juyou JUYOU_SRCS)
aux_source_directory(muxin MUXIN_SRCS)


set(OTHER_LIBRARIES common)
include_directories(${PROJECT_SOURCE_DIR}/common)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib) 

add_library(devices 
    ${JUYOU_SRCS} ${MUXIN_SRCS}
    )

target_link_libraries(devices 
    zmq ${PCL_LIBRARIES} ${OTHER_LIBRARIES}
    pthread
    )