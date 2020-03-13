#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>
#include "ros/ros.h"
#include <controller_manager/controller_manager.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_hw/kalman_filter.h>
#include <bmirobot_msg/Robot_distance.h>


#define ERR_EXIT(m) \
        do \
        { \
                perror(m); \
                exit(EXIT_FAILURE); \
        } while(0)
#define DEST_PORT1 8089
#define DEST_CAN_IP_ADDRESS "192.168.1.102"


using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ofstream proximityout;


int sock1,sock2;
struct sockaddr_in servaddr1;

float p1=0.0000001343;
float p2=0.00003673;
float p3=28.41;

main(int argc, char **argv)
{  
   int i;   
  ros::init(argc, argv, "bmirobot_proximity");

  ros::NodeHandle n("bmirobot");
  ros::Duration period(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher CAN_message=n.advertise<bmirobot_msg::Robot_distance>("CAN",1000);
  
  bmirobot_msg::Robot_distance fddistance;

  std::cout << argc <<"and"<< argv[1]<<'\n';



  int ret;
  int len;
  int temp;
  char sendbuf[1024] = {0};
  char recvbuf[1024] = {0};

  string saveflag;
  n.getParam("/savedata", saveflag);
   
  string deviceip;
  n.getParam("/deviceIP", deviceip);
    const char* deviceip1=deviceip.c_str();
//deviceip1=(char *)deviceip;char* aaa = append.c_str()
  std::cout<<deviceip1<<endl;

  memset(&servaddr1, 0, sizeof(servaddr1));

  if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket1");

  socklen_t addr_len = sizeof(struct sockaddr_in);
  servaddr1.sin_family = AF_INET;
  servaddr1.sin_port = htons(DEST_PORT1);
  servaddr1.sin_addr.s_addr =  htonl(INADDR_ANY);


  if(  bind(sock1,(struct sockaddr*)&servaddr1,sizeof(servaddr1))<0)
  {
 
    ERR_EXIT("bindsocket1");
  }

  

  
	time_t t = time(0);
	char tmp[100];
	struct tm *info;
	info = localtime(&t);
	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/Dropbox/Programs/record_data/%d-%d-%d-%d-%d-%d-can.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/new-can.csv");
	std::cout<<tmp<<endl;
	proximityout.open(tmp, std::ofstream::out | std::ofstream::binary);

  for(int i=0;i<3;i++)
  {
     sendbuf[0]=0x03;
     
     sleep(1);
    
     //ROS_INFO("step3 %d-%d-%d",i,i,i);
     if(ret !=0)
       break;
  }
  ROS_ERROR("start run");
  while (ros::ok())
  {  
      
   // ROS_INFO("start up");
    len= recvfrom(sock1,recvbuf,sizeof(recvbuf), 0, (struct sockaddr *)&servaddr1, &addr_len);

    
    if(len>0)
    {
      int j=0;    
      for(i=2;i<18;i=i+2)
			{ 
		
       
			 fddistance.proximity[j++]=(recvbuf[i]<<8)+recvbuf[i+1];
			
			}
       for(i=33;i>18;i=i-2)
			{ 
			
       
			 fddistance.proximity[j++]=(recvbuf[i-1]<<8)+recvbuf[i];
			
			}
       for(i=34;i<50;i=i+2)
			{ 

       
			 fddistance.proximity[j++]=(recvbuf[i]<<8)+recvbuf[i+1];
			
			}

    }
     //ROS_INFO("step3 %d-%d-%d",1,2,3);
 	for(int i=0;i<25;i++)
    {
        fddistance.realdistance[i]=float((p1*pow(fddistance.proximity[i],2)+p2*(fddistance.proximity[i])+p3));
        if (fddistance.realdistance[i]==28)
          { 
           fddistance.realdistance[i]=0;
          }
       
    }
     CAN_message.publish(fddistance);
     
     //ROS_INFO("sensor1: %6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d",len/2,fddistance.proximity[0],fddistance.proximity[1],fddistance.proximity[2],fddistance.proximity[3],fddistance.proximity[4],fddistance.proximity[5],fddistance.proximity[6],fddistance.proximity[7]);
     //ROS_INFO("sensor2: %d,%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x",len/2,0x0000ffff&((recvbuf[18]<<8)+recvbuf[19]),0x0000ffff&((recvbuf[20]<<8)+recvbuf[21]),0x0000ffff&((recvbuf[22]<<8)+recvbuf[23]),0x0000ffff&((recvbuf[24]<<8)+recvbuf[25]),0x0000ffff&((recvbuf[26]<<8)+recvbuf[27]),0x0000ffff&((recvbuf[28]<<8)+recvbuf[29]),0x0000ffff&((recvbuf[30]<<8)+recvbuf[31]),0x0000ffff&((recvbuf[32]<<8)+recvbuf[33]));
     //ROS_INFO("sensor2: %6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d",len/2,fddistance.proximity[8],fddistance.proximity[9],fddistance.proximity[10],fddistance.proximity[11],fddistance.proximity[12],fddistance.proximity[13],fddistance.proximity[14],fddistance.proximity[15]);
     //ROS_INFO("sensor3: %6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d",len/2,fddistance.proximity[16],fddistance.proximity[17],fddistance.proximity[18],fddistance.proximity[19],fddistance.proximity[20],fddistance.proximity[21],fddistance.proximity[22],fddistance.proximity[23]);


	ros::Time cantime = ros::Time::now();
		  	if(proximityout.is_open())
		  	{
			   	proximityout <<cantime<<",";
			
				  proximityout <<fddistance.proximity[0]<<","<<fddistance.proximity[1]<<","<<fddistance.proximity[2]<<","<<fddistance.proximity[3]<<","<<fddistance.proximity[4]<<","<<fddistance.proximity[5]<<","<<fddistance.proximity[6]<<","<<fddistance.proximity[7]<<",";
			    proximityout <<fddistance.proximity[8]<<","<<fddistance.proximity[9]<<","<<fddistance.proximity[10]<<","<<fddistance.proximity[11]<<","<<fddistance.proximity[12]<<","<<fddistance.proximity[13]<<","<<fddistance.proximity[14]<<","<<fddistance.proximity[15]<<",";
				  proximityout <<fddistance.proximity[16]<<","<<fddistance.proximity[17]<<","<<fddistance.proximity[18]<<","<<fddistance.proximity[19]<<","<<fddistance.proximity[20]<<","<<fddistance.proximity[21]<<","<<fddistance.proximity[22]<<","<<fddistance.proximity[23]<<",";
					proximityout<<endl;
		  	}

	 ros::spinOnce();
 }

 close(sock1);
 spinner.stop();
}
