cmake_minimum_required(VERSION 2.8)
project(birdview_vo)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(Thirdparty/filesystemSimplified)
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/line_descriptor)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

aux_source_directory(./src/ SRCS)
add_library(birdview_vo ${SRCS})
target_link_libraries(birdview_vo PUBLIC ${OpenCV_LIBS} ceres DBoW2 pangolin linedesc)
target_include_directories(birdview_vo PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(test_vo test/test_vo.cpp)
target_link_libraries(test_vo birdview_vo stlplus)

add_executable(test_ceres test/test_ceres.cpp src/SE2.cpp)
target_link_libraries(test_ceres ceres ${OpenCV_LIBS})

add_executable(test_line test/test_line.cpp src/KeyLineGeometry.cpp)
target_link_libraries(test_line ${OpenCV_LIBS} stlplus linedesc DBoW2)

add_executable(test_line_geometry test/test_line_geometry.cpp src/KeyLineGeometry.cpp)
target_link_libraries(test_line_geometry ${OpenCV_LIBS} linedesc DBoW2)