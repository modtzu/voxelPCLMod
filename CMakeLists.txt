CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
PROJECT(SimpleVoxelViewer)

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  ADD_DEFINITIONS(-std=c++11)
ENDIF()

FIND_PACKAGE(Voxel 0.1.0 REQUIRED)
find_package(PCL 1.6 REQUIRED COMPONENTS common io visualization)

add_definitions(${PCL_DEFINITIONS})

add_executable(SimpleVoxelViewer SimpleVoxelViewer.cpp PCLViewer.cpp)
target_include_directories(SimpleVoxelViewer PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${VOXEL_INCLUDE_DIRS})
target_link_libraries(SimpleVoxelViewer voxelpcl ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${VOXEL_LIBRARIES})


