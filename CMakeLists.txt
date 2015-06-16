cmake_minimum_required(VERSION 2.8.3)
project(image_entropy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

# find_package(Eigen REQUIRED)

# add_message_files(
#     FILES
#     WeightedPath.msg
#     Paths.msg
# )

generate_messages(
    DEPENDENCIES
    nav_msgs
    std_msgs
    geometry_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES topo_maps
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
   message_runtime tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(${EIGEN_INCLUDE_DIR})

## Declare a cpp library
# add_library(topo_maps
#   src/${PROJECT_NAME}/topo_maps.cpp
# )

## Declare a cpp executable
add_executable(image_entropy_node src/image_entropy_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(topo_maps_node topo_maps_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(image_entropy_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

# target_link_libraries(topo_maps_node
#    ${catkin_LIBRARIES} nrlevg
# )
#
# target_link_libraries(topo_send_image
#    ${catkin_LIBRARIES} nrlevg
# )
