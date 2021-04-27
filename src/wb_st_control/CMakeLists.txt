cmake_minimum_required(VERSION 2.8.3)
project(wb_st_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  wb_msgs
)

set(CMAKE_CXX_STANDARD 11)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES wb_st_control
  CATKIN_DEPENDS python3-numpy python3-yaml roscpp rospy wb_msgs
  #DEPENDS system_lib
)

file(COPY lib/pre_compiled_controller
  DESTINATION ${CATKIN_DEVEL_PREFIX}/lib/wb_st_control/ 
  FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                    GROUP_EXECUTE GROUP_READ)

