#include "ros/ros.h"
#include <bmirobot_hw/bmirobot.h>
#include <controller_manager/controller_manager.h>
main(int argc, char **argv)
{
  ros::init(argc, argv, "bmirobot_hw");

  ros::NodeHandle n("bmirobot");
  ros::Duration period(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  BMIRobot robot;
  controller_manager::ControllerManager cm(&robot,n);
  usleep(300);
  std::cout << argc << argv[1]<<'\n';
  double temp;
  double cmd[8];
  bool startF = true;
  int startCount=70;
  if(argc > 2)
  {
	  for(int i = 0;i < 8 ;i++)
	{
	  sscanf(argv[1+i],"%lf",&(cmd[i]));
	}
  }
  //temp = temp / 10;
  //robot.write(cmd);
  //return(0);
  ROS_ERROR("aaaaa");
  while (ros::ok())
  {
     if(startF == false)
     robot.read();
     if(startCount == 0)
     cm.update(robot.get_time(), robot.get_period());
     else
     {
	startCount --;
	  for(int i = 0;i < 8 ;i++)
	{
	  cmd[i] = 0;
	}
     }

     if(startCount == 0)
        robot.write(cmd);
     else
        robot.write(cmd,1);
     startF = false;
     usleep(50000);
     //ROS_ERROR("aaaaa");
  }
spinner.stop();
}
