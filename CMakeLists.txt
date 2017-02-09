cmake_minimum_required(VERSION 2.8.3)
project(wholesetup)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  genmsg
  message_generation
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FocusVal.msg
  PtuVals.msg
  ZoomVal.msg
#  TargetPoint.msg  
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
 # geometry_msgs
)
## catkin specific configuration ##
catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp std_msgs geometry_msgs message_runtime
)
###########
## Build ##
###########
# include_directories(include)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /wholesetup
)

 add_executable(imgProc_node src/imgProc.cpp)
 target_link_libraries(imgProc_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )
add_executable(focuszoomctrl_node src/focuszoomctrl.cpp)
target_link_libraries(focuszoomctrl_node
   ${catkin_LIBRARIES}
)
add_executable(ptuctrl_node src/ptuctrl.cpp
   src/linuxser.c
   src/ptu.c
)
target_link_libraries(ptuctrl_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############
