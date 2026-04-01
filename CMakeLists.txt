cmake_minimum_required(VERSION 3.10)

project(class_in_Tuesday_mac)

find_package(jsoncpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} 
            ${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS} 
            ${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS} 
            ${OpenCV_DEFINITIONS})

add_executable(class3-24 pj/pj3-24.cpp)
add_executable(class3-31 pj/pj3-31.cpp)

target_link_libraries(class3-24 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(class3-31 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})