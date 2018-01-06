cmake_minimum_required(VERSION 3.0)
project(pcl_pcd_recording_merger)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.54 COMPONENTS filesystem REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

include_directories(${CMAKE_HOME_DIRECTORY}/include)

set(SOURCE_FILES src/main.cpp src/Logger.cpp src/CameraExtrinsicsIO.cpp)

add_executable(pcl_pcd_recording_merger ${SOURCE_FILES})

target_link_libraries(pcl_pcd_recording_merger ${PCL_LIBRARIES})
target_link_libraries(pcl_pcd_recording_merger ${OpenCV_LIBS})
target_link_libraries(pcl_pcd_recording_merger ${Boost_LIBRARIES})

