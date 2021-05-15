cmake_minimum_required(VERSION 2.6)
aux_source_directory(time TIME_SRCS)
aux_source_directory(thread_pool TIME_POOL)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib) 
add_library(common 
         ${TIME_SRCS}
         ${TIME_POOL}
          )