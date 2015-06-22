cmake_minimum_required(VERSION 2.8.3)
project(image_entropy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Ofast -march=native -flto -fwhole-program")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

generate_messages(
    DEPENDENCIES
    nav_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
    message_runtime tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(image_entropy_node src/image_entropy_node.cpp)
add_executable(entropy_fuser_node src/entropy_fuser_node.cpp)

target_link_libraries(image_entropy_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

target_link_libraries(entropy_fuser_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
