# ROS_hack
A ROS reposity about flexible dual-arm.
## Simulation
- only load Gezebo model: roslaunch bmirobot_gazebo bmirobot_two_world.launch  
- load Gazebo and ros_controller: roslaunch bmirobot_gazebo bmirobot_gazebo_control.launch  
- communication between Gazebo and Rviz: roslaunch bmirobot_gazebo bmirobot_gazebo_moveit.launch  
<a>
    <img class="course-image" src="Figure/gazebo.png">
</a>
<a>
    <img class="course-image" src="Figure/rviz.png">
</a>   
- [trajectory planning under Moveit Framework: video link](Figure/rviz_gazebo.mp4)

## Real System
- launch right arm: roslaunch bmirobot_hw bmirobot_zhs_right.launch  
- launch dual-arm: roslaunch bmirobot_hw bmirobot_zhs_two.launch  
- simutaneously launch move_group node and rviz: roslaunch bmirobot_py bmirobot_rviz.launch  
- tele-operation using keyboard: rosrun bmirobot_py move_group_tele_key2.py  
<a>
    <img class="course-image" src="Figure/real_system.jpg">
</a>

## Tele-operation
- slave host: roslaunch omni_common omni_state.launch(ref: [Geomagic 3D touch driver](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers))  
- master host: rosrun bmirobot_py ik_grasp_touch.py  
- [tele-operation in simulation: video link](Figure/tele_op_touch.mp4)  
![best](Figure/cartpole.gif) <br />  

