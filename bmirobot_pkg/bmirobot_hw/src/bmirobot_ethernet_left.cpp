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
#include <fcntl.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_hw/kalman_filter.h>

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

using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ofstream mtfdout,mpuout;

static char sendcmd[1024] = {0};
int sock1,sock2;
struct sockaddr_in servaddr1;
struct sockaddr_in servaddr2;

static long ctrtimeout=0;
static int32_t watchdata;
static long statusTX=0;

uint16_t msgcount=0,mpucount=0;

int32_t oldpst[8]={0};
int32_t adpst[8]={0};
int16_t pstflag[8]={0};

void adjustpst(int16_t pst,int16_t index)
{
	int32_t temp=pst-oldpst[index]	;

	if(temp>18000)
	{
		pstflag[index]=pstflag[index]-1;
	}else
	if(temp<-18000)
	{
		pstflag[index]=pstflag[index]+1;
	}

	if(pstflag[index]==1)
	{
		adpst[index]=18000+(pst+18000);	
	}else if(pstflag[index]==-1)
	{
		adpst[index]=-18000+(pst-18000);
	}else
	{
		adpst[index]=pst;
	}
	oldpst[index]=pst;
	
}



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
	//ROS_ERROR("recieve left ctr : [%d]",msg->mtID[7]);
  	//watchdata=msg->mtpst[4];
  	//  ctrtimeout+=1;
  	//  ctrtimeout=ctrtimeout%400;

  	//	sendcmd[2+8*7]=(ctrtimeout&0xff);
  	//  sendcmd[3+8*7]=(ctrtimeout>>8)&0xff;
	//sendto(sock1, sendcmd, 64, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
    
    //ctrtimeout=0;
    //ROS_INFO("recieve: [%d]",msg->mtID[0]);
}

int sock_bind(int lisfd, int port)
{
	struct sockaddr_in myaddr;
	memset((char *)&myaddr, 0, sizeof(struct sockaddr_in));//清零
	myaddr.sin_family = AF_INET;//IPV4
	myaddr.sin_port = htons(port);//端口
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);//允许连接到所有本地地址上
	if (bind(lisfd, (struct sockaddr *)&myaddr, sizeof(struct sockaddr))==-1)
	{
		perror("sock_bind failed!\n");
		exit(1);
	}
	return 0;
}


main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_ethernet_driver");

	ros::NodeHandle n("bmirobot");
	ros::Duration period(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher Mt_fdctrpub = n.advertise<bmirobot_msg::Robot_fdctr>("lMT_fdctr", 100);
	ros::Publisher Mt_fdstatuspub = n.advertise<bmirobot_msg::Robot_fdstatus>("lMT_fdstatus", 100);
	ros::Publisher Mt_mpupub = n.advertise<bmirobot_msg::Robot_mpu>("lMT_fdmpu", 1000);
	ros::Subscriber MT_ctrsub = n.subscribe("lMT_ctr", 1000, CtrmsgCallback);

	bmirobot_msg::Robot_fdctr fdctrmsg;
	bmirobot_msg::Robot_fdstatus fdstatusmsg;
	bmirobot_msg::Robot_mpu fdmpu;

	std::cout << argc <<"and"<< argv[1]<<'\n';

	socklen_t socklen;
	int ret;
	char sendbuf[1024] = {0};
	char recvbuf[1024] = {0};
	char recvbuf2[1024] = {0};

	string deviceip;
	n.getParam("/left_IP", deviceip);

	int remoteport;
	n.getParam("/left_port", remoteport);

	string saveflag;
	n.getParam("/savedata", saveflag);

	const char* deviceip1=deviceip.c_str();
	//deviceip1=(char *)deviceip;char* aaa = append.c_str()
	std::cout<<remoteport<<endl;
	std::cout<<deviceip1<<endl;


	memset(&servaddr1, 0, sizeof(servaddr1));
	servaddr1.sin_family = AF_INET;
	servaddr1.sin_port = htons(remoteport);
	servaddr1.sin_addr.s_addr =  inet_addr(deviceip1);

	socklen = sizeof(servaddr1);

	//inet_pton(AF_INET, deviceip, &servaddr1.sin_addr.s_addr);
	//servaddr1.sin_addr.s_addr=deviceip;

	//memset(&servaddr2, 0, sizeof(servaddr2));
	//servaddr2.sin_family = AF_INET;
	//servaddr2.sin_port = htons(DEST_PORT2);
	//servaddr2.sin_addr.s_addr =  inet_addr(DSET_IP_ADDRESS);



  
	time_t t = time(0);
	char tmp[100];
	char* path;
	struct tm *info;
	info = localtime(&t);
	path = getcwd(NULL, 0);
	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/%d-%d-%d-%d-%d-%d-leftmtfd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/new-leftmtfd.csv");
	std::cout<<tmp<<"sdf"<<endl;
	mtfdout.open(tmp, std::ofstream::out | std::ofstream::binary);

	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/%d-%d-%d-%d-%d-%d-leftmpufd.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);	
	else
		sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/new-leftmpufd.csv");
	std::cout<<tmp<<endl;
	mpuout.open(tmp, std::ofstream::out | std::ofstream::binary);
  	
	if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket1");
	sock_bind(sock1, remoteport);
	int flag = fcntl(sock1, F_GETFL, 0);
	if (flag < 0)
	{
		perror("fcntl failed.\n");
		exit(1);
	}
	flag |= O_NONBLOCK;
	if (fcntl(sock1, F_SETFL, flag) < 0)
	{
		perror("fcntl failed.\n");
		exit(1);
	}

	for(int i=0;i<3;i++)
	{
		sendbuf[0]=0x03;
		ROS_ERROR("test connect");
		sendto(sock1, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
		//sendto(sock2, sendbuf, strlen(sendbuf), 0, (struct sockaddr *)&servaddr2, sizeof(servaddr2));
		sleep(1);
		ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&servaddr1, &socklen);
		if(ret !=-1)
			break;
	}

	ROS_ERROR("start run");
	while (ros::ok())
	{
			//ROS_INFO("recieve new start");
		int lnum=sendto(sock1, sendcmd, 64, 0, (struct sockaddr *)&servaddr1, sizeof(servaddr1));
		if(lnum == -1 )
		{
			perror("send left fail.\n");
		}else
		{
			//ROS_INFO("send left num:%d",lnum);
		}

		for(int i=0;i<2;i++)
		{
			ret = recvfrom(sock1, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&servaddr1, &socklen);
			//ROS_INFO("left sucessful:%d",ret);
			if (ret == 373)
			{
				recvbuf[ret]='\0';
				for(int i=0;i<8;i++)
				{
					fdstatusmsg.mt_mode[i]=(int16_t)(recvbuf[0+5+46*i]&0xff)+(recvbuf[1+5+46*i]&0xff)*0x100;

					fdstatusmsg.mt_Gpst[i]=(int16_t)((recvbuf[2+5+46*i]&0xff)+(recvbuf[3+5+46*i]&0xff)*0x100);
					fdstatusmsg.mt_Cpst[i]=(int16_t)((recvbuf[4+5+46*i]&0xff)+(recvbuf[5+5+46*i]&0xff)*0x100);
					fdstatusmsg.mt_Lpst[i]=(int16_t)((recvbuf[6+5+46*i]&0xff)+(recvbuf[7+5+46*i]&0xff)*0x100);

					fdstatusmsg.mt_Gspd[i]=(int16_t)((recvbuf[8+5+46*i]&0xff)+(recvbuf[9+5+46*i]&0xff)*0x100);
					fdstatusmsg.mt_Cspd[i]=(int16_t)((recvbuf[10+5+46*i]&0xff)+(recvbuf[11+5+46*i]&0xff)*0x100);
					fdstatusmsg.mt_Lspd[i]=(int16_t)((recvbuf[12+5+46*i]&0xff)+(recvbuf[13+5+46*i]&0xff)*0x100);

					fdstatusmsg.mt_Ctime[i]=(int32_t)(recvbuf[16+5+46*i]&0xff)+(recvbuf[17+5+46*i]&0xff)*0x100;
					fdstatusmsg.mt_Rtime[i]=(int32_t)(recvbuf[18+5+46*i]&0xff)+(recvbuf[19+5+46*i]&0xff)*0x100;

					fdstatusmsg.mt_Gtq[i]=(int16_t)(((recvbuf[21+5+46*i]&0xff)*0x100)+(recvbuf[20+5+46*i]&0xff));
					
					fdstatusmsg.mt_sysclk[i]=(int32_t)(recvbuf[22+5+46*i]&0xff)+(recvbuf[23+5+46*i]&0xff)*0x100+(recvbuf[24+5+46*i]&0xff)*0x10000+(recvbuf[25+5+46*i]&0xff)*0x1000000;
					fdstatusmsg.mt_smptime[i]=(int32_t)(recvbuf[26+5+46*i]&0xff)+(recvbuf[27+5+46*i]&0xff)*0x100;
					
					fdstatusmsg.mt_cputmp[i]=(int32_t)(recvbuf[28+5+46*i]&0xff)+(recvbuf[29+5+46*i]&0xff)*0x100;
					fdstatusmsg.mt_mttmp[i]=(int32_t)(recvbuf[30+5+46*i]&0xff)+(recvbuf[31+5+46*i]&0xff)*0x100;
					
					fdstatusmsg.mt_incrt[i]=(int16_t)((recvbuf[32+5+46*i]&0xff)+(recvbuf[33+5+46*i]&0xff)*0x100);
					fdstatusmsg.mt_invlt[i]=(int16_t)((recvbuf[34+5+46*i]&0xff)+(recvbuf[35+5+46*i]&0xff)*0x100);
					
					fdstatusmsg.mt_PWMduty[i]=(int16_t)((recvbuf[36+5+46*i]&0xff)+(recvbuf[37+5+46*i]&0xff)*0x100);
					fdstatusmsg.mt_PWMfrq[i]=(int16_t)((recvbuf[38+5+46*i]&0xff)+(recvbuf[39+5+46*i]&0xff)*0x100);
					
					fdstatusmsg.mt_ecd[i]=((recvbuf[41+5+46*i]*0x100))+(recvbuf[40+5+46*i]&0xff);
					fdstatusmsg.mt_ecdcnt[i]=(int32_t)(recvbuf[42+5+46*i]&0xff)+(recvbuf[43+5+46*i]&0xff)*0x100+(recvbuf[44+5+46*i]&0xff)*0x10000+(recvbuf[45+5+46*i]&0xff)*0x1000000;
					
					fdctrmsg.mt_mode[i]=	fdstatusmsg.mt_mode[i];
					fdctrmsg.mt_Cpst[i]=	fdstatusmsg.mt_Cpst[i];
					fdctrmsg.mt_Cspd[i]=	fdstatusmsg.mt_Cspd[i];
					fdctrmsg.mt_incrt[i]=	fdstatusmsg.mt_incrt[i];
					fdctrmsg.mt_PWMduty[i]=	fdstatusmsg.mt_PWMduty[i];

				}
				//msgcount++;
				ros::Time begin = ros::Time::now();
				if(mtfdout.is_open())
				{
					mtfdout <<begin<<",";
					for(int i=0;i<8;i++)
					{
						mtfdout <<fdctrmsg.mt_Cpst[i]<<","<<fdctrmsg.mt_Cspd[i]<<","<<fdctrmsg.mt_incrt[i]<<","<<fdctrmsg.mt_PWMduty[i]<<","<<fdstatusmsg.mt_ecd[i]<<",";
					}
					mtfdout<<endl;
				}
				//ROS_INFO("left :%d,%d,%d",ret,fdctrmsg.mt_mode[0],fdctrmsg.mt_Cpst[0]);
		
				for(int i=0;i<8;i++)
				{
					adjustpst(fdctrmsg.mt_Cpst[i],i);
					fdctrmsg.mt_Cpst[i]=adpst[i];
					//ROS_INFO("read:%d,%d",i,);	
				}

				Mt_fdctrpub.publish(fdctrmsg);
				ros::spinOnce();
				statusTX++;
				if(statusTX>100)
				{
					statusTX=0;
					Mt_fdstatuspub.publish(fdstatusmsg);

				}
			}
			if(ret==100)
			{
				recvbuf[ret]='\0';
				for(int i=0;i<7;i++)
				{
					fdmpu.mpu_Ax[i]=(int16_t)((recvbuf[0+7+12*i]&0xff)+(recvbuf[1+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Ay[i]=(int16_t)((recvbuf[2+7+12*i]&0xff)+(recvbuf[3+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Az[i]=(int16_t)((recvbuf[4+7+12*i]&0xff)+(recvbuf[5+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Rx[i]=(int16_t)((recvbuf[6+7+12*i]&0xff)+(recvbuf[7+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Ry[i]=(int16_t)((recvbuf[8+7+12*i]&0xff)+(recvbuf[9+7+12*i]&0xff)*0x100);
					fdmpu.mpu_Rz[i]=(int16_t)((recvbuf[10+7+12*i]&0xff)+(recvbuf[11+7+12*i]&0xff)*0x100);
				}
				Mt_mpupub.publish(fdmpu);
				ros::spinOnce();
				
				ros::Time mputime = ros::Time::now();
				if(mpuout.is_open())
				{
					mpuout <<mputime<<",";
					for(int i=0;i<7;i++)
					{
						mpuout <<fdmpu.mpu_Ax[i]<<","<<fdmpu.mpu_Ay[i]<<","<<fdmpu.mpu_Az[i]<<","<<fdmpu.mpu_Rx[i]<<","<<fdmpu.mpu_Ry[i]<<","<<fdmpu.mpu_Rz[i]<<",";
					}
					mpuout<<endl;
				}
			}
			//ROS_INFO("recv1 sucessful:%d",ret);
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
		ros::spinOnce();
        usleep(1000);
  }
  close(sock1);
  spinner.stop();
}

