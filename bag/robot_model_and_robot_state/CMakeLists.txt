add_executable(robot_model_and_robot_state_tutorial src/robot_model_and_robot_state_tutorial.cpp)
target_link_libraries(robot_model_and_robot_state_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS robot_model_and_robot_state_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(bmirobot_IK src/bmirobot_IK.cpp)
target_link_libraries(bmirobot_IK ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS bmirobot_IK DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(bmirobot_IK_touch src/bmirobot_IK_touch.cpp)
target_link_libraries(bmirobot_IK_touch ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS bmirobot_IK_touch DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
