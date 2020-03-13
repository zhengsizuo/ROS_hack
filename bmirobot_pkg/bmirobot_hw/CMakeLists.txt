cmake_minimum_required(VERSION 2.8.3)
project(bmirobot_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_manager 
  serial
  urdf
 ## eigen_conversions
  message_generation
  bmirobot_msg
  cmake_modules
)

find_package(Eigen REQUIRED) # 添加 2
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp
) 

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Robot_ctr.msg
#   Robot_fdctr.msg
#   Robot_fdstatus.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
##catkin_package(
##  INCLUDE_DIRS include
  
#  LIBRARIES bmirobot_hw
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} include
  
)

include_directories(${Eigen_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#    include/${PROJECT_NAME}/bmirobot.h
#    include/${PROJECT_NAME}/bmirobot_V4.h
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(bmirobot_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 	#add_executable(bmirobot_hw_node 		      src/main.cpp src/bmirobot.cpp)
 	add_executable(bmirobot_ethernet_driver   src/bmirobot_ethernet.cpp )
 	add_executable(bmileft_ethernet_driver    src/bmirobot_ethernet_left.cpp )
	add_executable(bmiright_ethernet_driver   src/bmirobot_ethernet_right.cpp )
	add_executable(bmirobotv4_hw 			        src/bmirobot_V4.cpp  src/kalman_filter.cpp )
 	add_executable(bmirobotv5_hw 			        src/bmirobot_V5.cpp)
	add_executable(bmirobotv51_hw 			      src/bmirobot_V51.cpp)
  add_executable(bmirobotv51_two 			      src/bmirobot_V51_main.cpp src/bmirobot_V51_right.cpp src/bmirobot_V51_left.cpp)
  add_executable(bmirobotv51_zhs 			      src/bmirobot_zhs_main.cpp src/bmirobot_zhs_right.cpp src/bmirobot_zhs_left.cpp)
	add_executable(bmirobotv52_hw 			      src/bmirobot_V52.cpp  )
  add_executable(bmirobotv5_test_hw 			  src/bmirobot_V5_test.cpp  )
  add_executable(bmirobot_proximity_driver  src/bmirobot_proximity.cpp)
    
    
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(bmirobot_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 #target_link_libraries(bmirobot_hw_node ${catkin_LIBRARIES})
 target_link_libraries(bmirobot_ethernet_driver   ${catkin_LIBRARIES} )
 target_link_libraries(bmileft_ethernet_driver    ${catkin_LIBRARIES} )
 target_link_libraries(bmiright_ethernet_driver   ${catkin_LIBRARIES} )
 target_link_libraries(bmirobotv4_hw ${catkin_LIBRARIES} )

 target_link_libraries(bmirobotv5_hw
   ${catkin_LIBRARIES}
 )
 target_link_libraries(bmirobotv51_hw           ${catkin_LIBRARIES} )
 target_link_libraries(bmirobotv51_two           ${catkin_LIBRARIES} )
 target_link_libraries(bmirobotv51_zhs           ${catkin_LIBRARIES} )
 target_link_libraries(bmirobotv52_hw        ${catkin_LIBRARIES} )
 target_link_libraries(bmirobotv5_test_hw
   ${catkin_LIBRARIES}
 )

 target_link_libraries(bmirobot_proximity_driver	
   ${catkin_LIBRARIES}
 ) 


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bmirobot_hw bmirobot_hw_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bmirobot_hw.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
