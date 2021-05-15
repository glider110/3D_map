cmake_minimum_required(VERSION 2.6)

set(VISUAL_ON OFF)

if (${VISUAL_ON} STREQUAL ON)
    add_definitions(-DVISUAL)
endif()


# set(CMAKE_CXX_STANDARD 11)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(OTHER_LIBRARIES common  devices)
include_directories(${PROJECT_SOURCE_DIR}/devices)
include_directories(${PROJECT_SOURCE_DIR}/common)

aux_source_directory(. DIR_SRCS)
aux_source_directory(pretreatment DIR_PRE)
aux_source_directory(cliff_detection DIR_CLIFF)
aux_source_directory(pass_through DIR_PASS)
aux_source_directory(slop_down DIR_SLOP_DOWN)


include_directories(obstacle_detection/process3D)
include_directories(obstacle_detection/tool3D)
include_directories(obstacle_detection)

aux_source_directory(obstacle_detection DIR_OBS)
aux_source_directory(obstacle_detection/process3D DIR_OBS_PRO)
aux_source_directory(obstacle_detection/tool3D DIR_OBS_3D)
aux_source_directory(obstacle_detection/render DIR_OBS_REN)


# message("chwchw"${PROJECT_SOURCE_DIR})
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib) 

add_library(process  ${DIR_SRCS} 
                    ${DIR_PRE} 
                    ${DIR_CLIFF}
                    ${DIR_PASS}
                    ${DIR_SLOP_DOWN}

                    ${DIR_OBS}
                    ${DIR_OBS_PRO}
                    ${DIR_OBS_3D}
                    ${DIR_OBS_REN}

                    )
target_link_libraries(process ${PCL_LIBRARIES} ${OTHER_LIBRARIES}
                    pthread                        
)
