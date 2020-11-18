cmake_minimum_required(VERSION 2.8.3)
project(hri_client)

######################################
## standard catkin find_package'ing ##
######################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  rosaria
  sound_play
  nav_msgs
  tf
)

## Custom messages used to command HRI related actions to the robots
add_message_files(
  FILES
  Action.msg
  ActionSequence.msg
)

# Generate ROS messages / services
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########################
## Thirdparty Dependency ##
###########################

# Extra project - libVISCA2
add_subdirectory(${PROJECT_SOURCE_DIR}/ThirdParty/libVISCA2)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Extra include directories for libvisca.h
include_directories(
  ${PROJECT_SOURCE_DIR}/ThirdParty/libVISCA2
)

## C++ libVISCA interface library
add_library(visca_interface
  src/visca_interface/visca_interface.cpp
)
add_dependencies(visca_interface
   visca
)
target_link_libraries(visca_interface
   visca
)

## C++ main HRI interface library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/hri_client.cpp
)
add_dependencies(${PROJECT_NAME} 
   visca_interface
   ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}
   visca_interface
   ${catkin_LIBRARIES}
)

## ROS node executable
add_executable(${PROJECT_NAME}_node src/nodes/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Other files for installation
install(FILES
  launch/hri_client.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
