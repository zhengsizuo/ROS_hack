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
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)

#define ERR_EXIT(m) \
        do \
        { \
                perror(m); \
                exit(EXIT_FAILURE); \
        } while(0)
#define DEST_PORT1 10000
#define DEST_PORT2 20000
#define DSET_IP_ADDRESS  "192.168.1.181"


static char sendcmd[1024] = {0};
int sock1,sock2;
struct sockaddr_in servaddr1;
struct sockaddr_in servaddr2;

static long ctrtimeout=0;
static int32_t watchdata;

void CtrmsgCallback(const bmirobot_msg::Robot_ctr::ConstPtr& msg)
{
    for(int i=0;i<8;i++)
    {
       	sendcmd[0+8*i]=(msg->mtID[i]&0xff);
        sendcmd[1+8*i]=(msg->mtmode[i]&0xff);
       	sendcmd[2+8*i]=(msg->mtpst[i]&0xff);
        sendcmd[3+8*i]=(msg->mtpst[i]>>8)&0xff;
       	sendcmd[4+8*i]=(msg->mtspd[i]&0xff);
        sendcmd[5+8*i]=(msg->mtspd[i]>>8)&0xff;
        sendcmd[6+8*i]=(msg->mttq[i]&0xff);
        sendcmd[7+8*i]=((msg->mttq[i])>>8)&0xff;
    }
  	//watchdata=msg->mtpst[4];
  	//  ctrtimeout+=1;
  	//  ctrtimeout=ctrtimeout%400;

  	//	sendcmd[2+8*7]=(ctrtimeout&0xff);
  	//  sendcmd[3+8*7]=(ctrtimeout>>8)&0xff;
	//sendto(sock1, sendcmd, 64, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
    
    //ctrtimeout=0;
    ROS_INFO("new motor command: [%d]",sendcmd[1+8*7]);
}

main(int argc, char **argv)
{
  ros::init(argc, argv, "bmirobot_ethernet_driver");

  ros::NodeHandle n("bmirobot");
  ros::Duration period(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher Mt_fdctrpub = n.advertise<bmirobot_msg::Robot_fdctr>("MT_fdctr", 1);
  //ros::Publisher Mt_fdstatuspub = n.advertise<bmirobot_msg::Robot_fdstatus>("MT_fdstatus", 1000);
  //ros::Publisher Mt_mpupub = n.advertise<bmirobot_msg::Robot_mpu>("MT_fdmpu", 1000);
  ros::Subscriber MT_ctrsub = n.subscribe("MT_ctr", 1000, CtrmsgCallback);

  bmirobot_msg::Robot_fdctr fdctrmsg;
  bmirobot_msg::Robot_fdstatus fdstatusmsg;
  bmirobot_msg::Robot_mpu fdmpu;

  std::cout << argc <<"and"<< argv[1]<<'\n';

  int ret;
  char sendbuf[1024] = {0};
  char recvbuf[1024] = {0};
  char recvbuf2[1024] = {0};

  memset(&servaddr1, 0, sizeof(servaddr1));
  servaddr1.sin_family = AF_INET;
  servaddr1.sin_port = htons(DEST_PORT1);
  servaddr1.sin_addr.s_addr =  inet_addr(DSET_IP_ADDRESS);


  memset(&servaddr2, 0, sizeof(servaddr2));
  servaddr2.sin_family = AF_INET;
  servaddr2.sin_port = htons(DEST_PORT2);
  servaddr2.sin_addr.s_addr =  inet_addr(DSET_IP_ADDRESS);


  if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket1");
  if ((sock2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket2");

  
  for(int i=0;i<3;i++)
  {
     sendbuf[0]=0x03;
     ROS_ERROR("test connect");
     sendto(sock1, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
     //sendto(sock2, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
     sleep(1);
     ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, NULL, NULL);
     if(ret !=0)
       break;
  }
     
  ROS_ERROR("start run");
  while (ros::ok())
  {
        //ROS_INFO("recieve new start");
	sendto(sock1, sendcmd, 64, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
 		
	for(int i=0;i<2;i++)
	{
	    ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, NULL, NULL);
	    if (ret == 373)
	    {
	        recvbuf[ret]='\0';
	        for(int i=0;i<8;i++)
	        {
	            fdctrmsg.mt_mode[i]=(int16_t)(recvbuf[0+5+46*i]&0xff)+(recvbuf[1+5+46*i]&0xff)*0x100;

	            fdctrmsg.mt_Gpst[i]=(int16_t)((recvbuf[2+5+46*i]&0xff)+(recvbuf[3+5+46*i]&0xff)*0x100);
	            fdctrmsg.mt_Cpst[i]=(int16_t)((recvbuf[4+5+46*i]&0xff)+(recvbuf[5+5+46*i]&0xff)*0x100);
	            fdctrmsg.mt_Lpst[i]=(int16_t)((recvbuf[6+5+46*i]&0xff)+(recvbuf[7+5+46*i]&0xff)*0x100);

	            fdctrmsg.mt_Gspd[i]=(int16_t)((recvbuf[8+5+46*i]&0xff)+(recvbuf[9+5+46*i]&0xff)*0x100);
	            fdctrmsg.mt_Cspd[i]=(int16_t)((recvbuf[10+5+46*i]&0xff)+(recvbuf[11+5+46*i]&0xff)*0x100);
	            fdctrmsg.mt_Lspd[i]=(int16_t)((recvbuf[12+5+46*i]&0xff)+(recvbuf[13+5+46*i]&0xff)*0x100);

	            fdctrmsg.mt_Ctime[i]=(int32_t)(recvbuf[16+5+46*i]&0xff)+(recvbuf[17+5+46*i]&0xff)*0x100;
	            fdctrmsg.mt_Rtime[i]=(int32_t)(recvbuf[18+5+46*i]&0xff)+(recvbuf[19+5+46*i]&0xff)*0x100;

	            fdctrmsg.mt_Gtq[i]=(int16_t)(((recvbuf[21+5+46*i]&0xff)*0x100)+(recvbuf[20+5+46*i]&0xff));

	            fdctrmsg.mt_sysclk[i]=(int32_t)(recvbuf[22+5+46*i]&0xff)+(recvbuf[23+5+46*i]&0xff)*0x100+(recvbuf[24+5+46*i]&0xff)*0x10000+(recvbuf[25+5+46*i]&0xff)*0x1000000;
	            fdctrmsg.mt_smptime[i]=(int32_t)(recvbuf[26+5+46*i]&0xff)+(recvbuf[27+5+46*i]&0xff)*0x100;

	            fdctrmsg.mt_cputmp[i]=(int32_t)(recvbuf[28+5+46*i]&0xff)+(recvbuf[29+5+46*i]&0xff)*0x100;
	            fdctrmsg.mt_mttmp[i]=(int32_t)(recvbuf[30+5+46*i]&0xff)+(recvbuf[31+5+46*i]&0xff)*0x100;

	            fdctrmsg.mt_incrt[i]=(int16_t)((recvbuf[32+5+46*i]&0xff)+(recvbuf[33+5+46*i]&0xff)*0x100);
	            fdctrmsg.mt_invlt[i]=(int16_t)((recvbuf[34+5+46*i]&0xff)+(recvbuf[35+5+46*i]&0xff)*0x100);
	            
	            fdctrmsg.mt_PWMduty[i]=(int16_t)((recvbuf[36+5+46*i]&0xff)+(recvbuf[37+5+46*i]&0xff)*0x100);
	            fdctrmsg.mt_PWMfrq[i]=(int16_t)((recvbuf[38+5+46*i]&0xff)+(recvbuf[39+5+46*i]&0xff)*0x100);
				
	            fdctrmsg.mt_ecd[i]=((recvbuf[41+5+46*i]*0x100))+(recvbuf[40+5+46*i]&0xff);
	            fdctrmsg.mt_ecdcnt[i]=(int32_t)(recvbuf[42+5+46*i]&0xff)+(recvbuf[43+5+46*i]&0xff)*0x100+(recvbuf[44+5+46*i]&0xff)*0x10000+(recvbuf[45+5+46*i]&0xff)*0x1000000;
	        }
	   		
	        //ROS_INFO("recv1 sucessful:%d,%8x,%8x,%8x,%8x,%8x",ret,fdctrmsg.mt_ecdcnt[0],recvbuf[42+5+46*0],recvbuf[43+5+46*0],recvbuf[44+5+46*0],recvbuf[45+5+46*0]);

	        Mt_fdctrpub.publish(fdctrmsg);
	        //Mt_fdstatuspub.publish(fdstatusmsg);
	    }
		if(ret==100)
		{
	        recvbuf[ret]='\0';
	        for(int i=0;i<8;i++)
	        {
	            fdmpu.mpu_Ax[i]= (int32_t)recvbuf2[0+7+12*i]+(int32_t)recvbuf2[1+7+12*i]*0x100;
	            fdmpu.mpu_Ay[i]= (int32_t)recvbuf2[2+7+12*i]+(int32_t)recvbuf2[3+7+12*i]*0x100;
	            fdmpu.mpu_Az[i]= (int32_t)recvbuf2[4+7+12*i]+(int32_t)recvbuf2[5+7+12*i]*0x100;
	            fdmpu.mpu_Rx[i]= (int32_t)recvbuf2[6+7+12*i]+(int32_t)recvbuf2[7+7+12*i]*0x100;
	            fdmpu.mpu_Ry[i]= (int32_t)recvbuf2[8+7+12*i]+(int32_t)recvbuf2[9+7+12*i]*0x100;
	            fdmpu.mpu_Rz[i]= (int32_t)recvbuf2[10+7+12*i]+(int32_t)recvbuf2[11+7+12*i]*0x100;
	        }
		}
		ROS_INFO("recv1 sucessful:%d",ret);
	}
	

        //ROS_INFO("recieve2 start");
        //ret = recvfrom(sock2, recvbuf2, sizeof(recvbuf2), 0, NULL, NULL);
        if (ret == -1)
        {
	    	ROS_INFO("recv2 failed");
        }else
        {
           // ROS_INFO("recv2 sucessful:%d,",ret);
        }

        /* if(ctrtimeout>=100)
        {
            for(int i=0;i<8;i++)
            {
                sendcmd[1+8*i]=0x02;
            }
            sendto(sock2, sendcmd, 64, 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
        }
        */
		//ROS_INFO("test:%d,%d",watchdata,fdctrmsg.mt_Gpst[4]);
		//ROS_INFO("test:%d,%d",ctrtimeout,fdctrmsg.mt_Gpst[7]);
        //usleep(1000);
  }
  close(sock1);
  spinner.stop();
}

