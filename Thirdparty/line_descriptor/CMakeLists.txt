project( line-descriptor )
cmake_minimum_required(VERSION 2.7)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED)
MESSAGE("-- OpenCV include dir: " ${OpenCV_INCLUDE_DIRS})

#if(COMMAND cmake_policy)
#    cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -O3 -mtune=native -march=native")

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB_RECURSE all_source_files src/*.cpp src/*.hpp)

add_library( linedesc ${all_source_files} )
target_include_directories(linedesc PUBLIC include PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries( linedesc PRIVATE ${OpenCV_LIBS} )

install(TARGETS linedesc DESTINATION lib)
install(FILES include/line_descriptor_custom.hpp DESTINATION include)
install(DIRECTORY include/line_descriptor DESTINATION include)
