cmake_minimum_required(VERSION 3.1)

project( class_in_Tuesday )

find_package(OpenCV REQUIRED)  
find_package(PCL 1.3 REQUIRED)  

# 将PCL的包含目录添加到项目的包含目录列表中，以便能够找到头文件  
include_directories( ${PCL_INCLUDE_DIRS} 
                     ${OpenCV_INCLUDE_DIRS} )  

# 将PCL的库目录添加到链接器搜索路径中  
link_directories( ${PCL_LIBRARY_DIRS}
                  ${OpenCV_LIBRARY_DIRS} )  
  
add_definitions(  ${PCL_DEFINITIONS}
                  ${OpenCV_DEFINITIONS} )  
  
# 添加一个可执行文件目标,例：名为pcd_write_test，源文件为pcd_write.cpp  
add_executable(test_one one.cpp)  
add_executable(test_two second.cpp)  
add_executable(test_three third.cpp)  
  
# 将PCL的库链接到pcd_write_test可执行文件目标。这里${PCL_LIBRARIES}包含了PCL及其依赖的所有库  
target_link_libraries(test_one ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(test_two ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(test_three ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})