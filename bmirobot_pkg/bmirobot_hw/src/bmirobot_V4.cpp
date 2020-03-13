#include <bmirobot_hw/bmirobot_V4.h>
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>

#include <urdf/model.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_msg/Robot_jointfd.h>
#include <stdlib.h>
#include <controller_manager/controller_manager.h>
#include <bmirobot_hw/kalman_filter.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <Eigen/Eigen>

#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)
#define pi          3.1415926
using namespace std;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

#define USE_ODOMETER    0
#define Motor_mode      0x00   //motor mode: (0x00 position) (0x04 speed) (0x08 torque) 0x0c forwardcontorl
#define Motor_status    0x02   //motor status (0x00 normal) (0x02 free) (0x03  hold)

#define pst_mode      0x00
#define spd_mode      0x04
#define tq_mode       0x08
#define fw_mode       0x0c

#define normal    0x00
#define free      0x02
#define hold      0x03

#define M1_ctr    	normal	+pst_mode
#define M2_ctr      normal	+pst_mode
#define M3_ctr      normal	+pst_mode
#define M4_ctr      normal	+fw_mode
#define M5_ctr      normal	+fw_mode
#define M6_ctr      normal	+pst_mode
#define M7_ctr      normal	+pst_mode
#define M8_ctr      normal	+fw_mode


int32_t sysstatus=0xff;

int direction[8]={1,1,1,1,1,1,1,1};

bmirobot_msg::Robot_ctr MTctrmsg;
bmirobot_msg::Robot_jointfd Jointfdmsg;
ros::Publisher Joint_fdpub;
ros::Publisher MT_ctrpub;

int32_t feedbackStart=0;
int32_t initcmd=0;

ofstream ctrout,pidout;

float home_pos[9]={0.0/180*pi, 0.0, 00.0/180*pi , 0.0/180*pi, 0.0 , 0, 0, 0, 0};

int32_t Mt_mode[8];
int32_t Mt_fdpst[8];
int32_t Mt_fdspd[8];
int32_t Mt_fdcrt[8];
int32_t Mt_fdpwm[8];
int32_t Mt_fdtq[8];
int32_t Mt_abecd[8];
int32_t Mt_rlecd[8];

int32_t mpuAx[8],mpuAy[8],mpuAz[8],mpuRx[8],mpuRy[8],mpuRz[8];

int32_t MpstAng[8];
int32_t MspdAng[8];
int32_t MtqNm[8];

double mt_vel[8]={0};
double cmd_updata[9]={0};


//kalman2_state kalman[8];


double gears[8]={1,1,1,1,0,0,0,0};
float joint_limition[9][2];
Eigen::Matrix<double, 8, 8> M2J;
Eigen::Matrix<double, 8, 1> J_vect, M_vect,M_cmd;

using namespace std;

double readAangle(unsigned char H,unsigned char L,int homepos=0)
{
  double a;
  return a;
}

double savehandmotor(double goalpst, double fdpst, int crt)
{
	static double diffupdate=0;
	double diff=abs(goalpst-fdpst);
	if(diff>0.2)
	{
		if(crt>700)
		{
			diffupdate+=(fdpst-goalpst)*0.01;		
		}else
		{
			
		}
	}else
	{
		diffupdate=0;
	}
	return (goalpst+diffupdate);
}

////// torque mode
//static double tqP[8]={1,  		1,   		1,			10000, 		10000,    	500,    500,      	0};
//static double tqI[8]={0.001,    0.001,     	0.001,  	1.0,   		1.0,  		1,  	1,      0};
//static double tqD[8]={4,  		4,   		4,			40, 		40,         100,    100,      	2};

///// pwm mode 
static double tqP[8]={1,  		1,   		1,			3000, 	  3000,    	1,    1,      1000};
static double tqI[8]={0.001,    0.001,     	0.001,  	20.0,     20.0,  		1,    1,      5};
static double tqD[8]={4,  		4,   		4,			200.0,    200.0,        1,    1,      50};

/***
static double tqP[8]={1,  		1,   		1,			8000, 	  8000,    	1,    1,      	0};
static double tqI[8]={0.001,    0.001,     	0.001,  	28.0,     28.0,  		1,    1,      0};
static double tqD[8]={4,  		4,   		4,			30.0, 	  30.0,         1,    1,      	2};
***/

static double sP[8]={1,  		1,   		1,			30000, 	  30000,    	1,    1,      	0};
static double sI[8]={0.001,    0.001,       0.001,  	50.0,     50.0,  		1,    1,        0};
static double sD[8]={4,  		4,   		4,			30.0, 	  30.0,         1,    1,      	2};

static double tqu[8]={0};
static double ipErr[8]={0}, pErr[8]={0},lastgp[8]={0};
static double ivErr[8]={0}, vErr[8]={0}, lvErr[8]={0},dvErr[8]={0};
static double PIDp[8]={0}, PIDi[8]={0},PIDd[8]={0};

double motor_tqPID(int num,double goalpst,double position, double speed, double current)
{
    double spdu=0,tquTemp=0;
    double errtqu=0;
    double step=100;
    double tqbound=3000.0;
	
	
    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
	PIDp[num]=tqP[num]*pErr[num];
	PIDd[num]=tqD[num]*(goalpst-lastgp[num]-speed);
	PIDi[num]=tqI[num]*ipErr[num];

    tqu[num] = PIDp[num]+PIDi[num] +PIDd[num];
	
	lastgp[num]=goalpst;
    //ROS_INFO("motor torqe:%2d ,tqu:%12f, gpst:%12f, pst:%12f, spd:%12f, crt:%12f, pE:%10f, ipE:%14f",num,tqu[num],goalpst, position, speed, current, pErr[num], ipErr[num]);
	
    //ROS_INFO("motor torqe:,%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f, %14f, %14f",tqu[num], spdu, speed, current, pwmduty, pErr[num], vErr[num], dvErr[num], ivErr[num], ipErr[num]);
	
	if(tqu[num]>5000)
		tqu[num]=5000;
	if(tqu[num]<-5000)
		tqu[num]=-5000;
    return (double)(tqu[num]);
}

double motor_pwmPID(int num,double goalpst,double position, double speed, double current)
{
    double spdu=0,tquTemp=0;
    double errtqu=0;
    double step=100;
    double tqbound=3000.0;

	
    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
	PIDp[num]=tqP[num]*pErr[num];
	PIDd[num]=tqD[num]*(goalpst-lastgp[num]-speed);
	PIDi[num]=tqI[num]*ipErr[num];

    tqu[num] = PIDp[num]+PIDi[num] +PIDd[num];
	lastgp[num]=goalpst;

	if(tqu[num]>5000)
		tqu[num]=5000;
	if(tqu[num]<-5000)
		tqu[num]=-5000;
    return (double)(tqu[num]);
}


char handJN[][13]={"hand_joint_1","hand_joint_4"};
BMIRobot::~BMIRobot() 
{

}

BMIRobot::BMIRobot() 
{

   //read robot limitions
   ros::NodeHandle node;
   std::string robot_desc_string;
   
   node.param("robot_description", robot_desc_string, std::string());

   urdf::Model model;
   //if (!model.initFile("/home/bmi/Documents/ws_moveit/src/bmirobot_pkg/bmirobot_description/urdf/bmirobot.xacro")){
   if (!model.initString(robot_desc_string)){
     	ROS_ERROR("Failed to parse urdf file");
    return;
   }
   std::cout<< "parse urdf file ok"<<std::endl;
   boost::shared_ptr<const urdf::Link> link = model.getLink("base_link");
   std::cout<< link->child_joints[0]->name << " \n";
   joint_limition[0][0] = link->child_joints[0]->limits->lower;
   joint_limition[0][1] = link->child_joints[0]->limits->upper;

   for(int i = 0;i<7;i++)
   {
		if(i==0)
     		link = link->child_links[0];
		else
			link = link->child_links[0];

        int index = 0;
    	if(i == 6)
		index = 1;

    	std::cout<< link->child_joints[index]->name << "   \n";
        joint_limition[i+1][0] = link->child_joints[index]->limits->lower;
        joint_limition[i+1][1] = link->child_joints[index]->limits->upper;
  }
  joint_limition[8][0] = -1*joint_limition[7][1];
  joint_limition[8][1] = -1*joint_limition[7][0];
  //exit(0);
  //joint_limition

    M2J<<   0,  		0,			0,   		0.5,    	0.5,     	0,      	0,      0,
            0, 			0,			0,   		-0.5,    	0.5,     	0,      	0,      0,
            0,  		0,   	    0,			0,			0,      	0.5,      	0.5,    0,
            0,      	0,      	0,			0,   		0,   		0.5,      	-0.5,    0,
            1,      	0,      	0,      	0,			0,   		0,      	0,      0,
            0,      	1,      	0,      	0,      	0,      	0,   		0,   	0,
            0,      	0,      	1,      	0,      	0,      	0,   		0,   	0,
            0,      	0,      	0,      	0,      	0,      	0,      	0,      1;

    // connect and register the joint state interface
    for(int i=0;i < JOINT_NUM+HAND_JOINT_NUM;i++)
    {
      	pos[i] = cmd[i] =0;
        vel[i] = 0;
        eff[i] = 0;
    }
    char jointName[255];
    char baseName[]="joint";

    //int i = 0;
    for(int i =0;i < JOINT_NUM;i++)
    {
     	sprintf(jointName,"%s%i",baseName,i+1);
        printf("%s\n",jointName);
     	hardware_interface::JointStateHandle temp(jointName, &pos[i], &vel[i], &eff[i]);
     	jnt_state_interface.registerHandle(temp);
    }
    for(int i =0;i < HAND_JOINT_NUM;i++)
    {
     	hardware_interface::JointStateHandle temp(handJN[i], &pos[i+JOINT_NUM], &vel[i+JOINT_NUM], &eff[i+JOINT_NUM]);
     	jnt_state_interface.registerHandle(temp);
    }
    registerInterface(&jnt_state_interface);

    for(int i =0;i < JOINT_NUM;i++)
    {
     	sprintf(jointName,"%s%i",baseName,i+1);
     	//connect and register the joint position interface
     	hardware_interface::JointHandle temp(jnt_state_interface.getHandle(jointName), &cmd[i]);
     	jnt_pos_interface.registerHandle(temp);
    }

    for(int i =0;i < HAND_JOINT_NUM;i++)
    {
     	hardware_interface::JointHandle temp(jnt_state_interface.getHandle(handJN[i]), &cmd[i+JOINT_NUM]);
     	jnt_pos_interface.registerHandle(temp);
    }

    registerInterface(&jnt_pos_interface);
    last_time = ros::Time::now();
    initrobot();
}

void BMIRobot::initrobot()
{
    ROS_INFO("finish initrobot---------------------");
}

void BMIRobot::read()
{
	unsigned char motorP[16];
	unsigned char head[2];


	for(int i=0;i < JOINT_NUM+1;i++)
	{
	 	//pos[i] = cmd[i];
	 	//vel[i] = 1;
	 	//eff[i] = 1;*/
		M_vect(i)= Jointfdmsg.Joint_fdpst[i];
	}
	// ROS_INFO("finish read data----------------------");
	J_vect = M2J*M_vect;
	
	for(int i=0;i < JOINT_NUM+1;i++)
	{
		// pos[i] =cmd[i]+random()%100/1000.0;//;
		pos[i]= J_vect(i)*direction[i]-home_pos[i];
		//pos[i]=cmd_updata[i];
	}
  
	pos[JOINT_NUM+1] = -1*pos[JOINT_NUM];
	//ROS_INFO("pos:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],pos[7],pos[8]);

	if(initcmd==0)
	{
		for(int i=0;i < JOINT_NUM+1;i++)
	    {
			cmd_updata[i]=J_vect(i)*direction[i]-home_pos[i];
			//cmd_updata[i]=pos[i];
		}
		initcmd=1;
	}

	//float *data = NULL;
	//data[0]
	//kalman1_filter(&state[0], data[i]);
}

int32_t testcount=0;
int32_t tstep=0;
ros::Time init_time ;
ros::Time now_time ;
void BMIRobot::write()
{
    double pstctr,speedctr,torquectr,PWMctr;

	float step = 0.02;

	float maxV = 0;
	int maxI=0;
	float diff[JOINT_NUM+2];
	double diftime;

	switch(tstep)
	{
		case 0:  // initial
			tstep=6;
			init_time= ros::Time::now();
		break;
		
		case 1:  // stable step
			now_time=ros::Time::now();
			diftime=(now_time-init_time).toSec();
			if(diftime>2)
			{
				tstep=2;
				init_time= ros::Time::now();
			}else
			{
				cmd[0]=0;
				cmd[1]=0;
			}
		break;

		case 2:
			now_time=ros::Time::now();
			diftime=(now_time-init_time).toSec();
			if(diftime>4)
			{
				init_time= ros::Time::now();
				tstep=3;
			}else{
				cmd[0]=(45.0/180*pi)*(sin((diftime)/4*2*pi));
				cmd[1]=(40.0/180*pi)*(cos((diftime)/4*2*pi)-1);
				//cmd[2]=pi/2+(55.0/180*pi)*(cos((diftime-3)/4*2*pi)-1);
				//cmd[3]=(45.0/180*pi)*(cos((diftime-3)/4*2*pi)-1);

			}
		break;
		case 3:
			now_time=ros::Time::now();
			diftime=(now_time-init_time).toSec();	
			if(diftime>2)
			{
				init_time= ros::Time::now();
				tstep=4;
			}
			

		break;
		case 4:
			now_time=ros::Time::now();
			diftime=(now_time-init_time).toSec();	
			if(diftime>2)
			{
				init_time= ros::Time::now();
				tstep=5;
			}
			cmd[0]=0;
			cmd[1]=0;
		break;
	}

    //ROS_INFO("cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7]);

  /*  for(int i=0;i < JOINT_NUM+2;i++)
    {
        diff[i] = cmd[i] - cmd_updata[i];
        if(abs(diff[i]) > maxV)
		{
			maxV = abs(diff[i]);
			maxI = i;
		}
    }

	if(maxV > step)
	{
	    for(int i=0;i < JOINT_NUM+2;i++)
	    {
	      cmd_updata[i] = cmd_updata[i]+diff[i]/maxV*step;
			//printf("adding,%d,,,,%f\n",i,diff[i]/maxV*step);
	    }
    }else
	{	
	    for(int i=0;i < JOINT_NUM+2;i++)
	    {
			cmd_updata[i]=cmd_updata[i]+diff[i];
		}
	}
*/

	//ROS_INFO("diff:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",diff[0],diff[1],diff[2],diff[3],diff[4],diff[5],diff[6],diff[7]);
   // ROS_INFO("changed_cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",cmd_updata[0],cmd_updata[1],cmd_updata[2],cmd_updata[3],cmd_updata[4],cmd_updata[5],cmd_updata[6],cmd_updata[7]);

    for(int i=0;i < JOINT_NUM+1;i++)
    {
        //J_vect(i) = (cmd_updata[i]+home_pos[i])*direction[i];
		J_vect(i) = (cmd[i]+home_pos[i])*direction[i];
    }

    unsigned char motroCmd[JOINT_NUM*2+2];
    M_vect= M2J.inverse()*J_vect;
    M_cmd =M_vect;
    ROS_INFO("M_vect:%f,%f,%f,%f,%f,%f,%f,%f",M_vect(0),M_vect(1),M_vect(2),M_vect(3),M_vect(4),M_vect(5),M_vect(6),M_vect(7));

    for(int i=0;i < 7+1;i++)
    {
		switch(MTctrmsg.mtmode[i]&0x0c)
		{
			case pst_mode:
					MTctrmsg.mtpst[i]=M_cmd(i)*(18000.0/pi);
					//pstctr=	motor_pstPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
					//MTctrmsg.mtpst[i]=(int32_t)(pstctr/pi*18000.0);
				break;
			case spd_mode:
				break;
			case tq_mode:
       				//torquectr = motor_tqPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Jointfdmsg.Joint_fdctr[i]);
     				MTctrmsg.mttq[i] =(int32_t) torquectr;
				break;
			case fw_mode:
					torquectr = motor_pwmPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Jointfdmsg.Joint_fdctr[i]);
     				MTctrmsg.mtpst[i] =(int32_t) torquectr;
				break;
		}
		//ROS_INFO("mtmode:%x,   %x",MTctrmsg.mtmode[i],MTctrmsg.mtmode[i]&0xc0);
    }
	//MTctrmsg.mtpst[7]=savehandmotor(M_cmd(7),Jointfdmsg.Joint_fdpst[7],Mt_fdcrt[7]);
	//testcount++;
	//MTctrmsg.mttq[0]=testcount;
	//ROS_INFO("test:%d",testcount);
    MT_ctrpub.publish(MTctrmsg);

	ros::Time begin = ros::Time::now();
  	if(ctrout.is_open())
  	{
	   	ctrout <<begin<<",";
		for(int i=0;i<8;i++)
		{
			ctrout <<M_vect(i)*(18000.0/pi)<<","<<Mt_fdpst[i]<<","<<Mt_fdspd[i]<<","<<Mt_fdcrt[i]<<",";	
		}
		ctrout<<endl; 
  	}

  	if(pidout.is_open())
  	{
	   	pidout <<begin<<",";
		for(int i=0;i<8;i++)
		{
			pidout <<tqu[i]<<","<<PIDp[i]<<","<<PIDi[i]<<","<<PIDd[i]<<","<<pErr[i]<<",";	
		}
		pidout<<endl; 
  	}


}

ros::Time BMIRobot::get_time(){
    return ros::Time::now();
}

ros::Duration BMIRobot::get_period(){
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time;
    last_time = current_time;
    return period;
}

void  FdctrmsgCB(const bmirobot_msg::Robot_fdctr::ConstPtr& msg)
{
	int32_t test=0;
    for(int i=0;i<8;i++)
    {
        Mt_mode[i]=msg->mt_mode[i];
        Mt_fdpst[i]= msg->mt_Cpst[i];
        Mt_fdspd[i]=(msg->mt_Cspd[i]);
        Mt_fdcrt[i]=(msg->mt_incrt[i]);
		//Mt_fdpwm[i]=(msg->mt_PWMduty[i]);
        // Mt_fdtq[i]=(msg->mt_Gtq[i]);
        //Mt_abecd[i]=(msg->mt_ecd[i]);
        //Mt_rlecd[i]=(msg->mt_ecdcnt[i]);
		test+=msg->mt_incrt[i];
    }
    for(int i=0;i<8;i++)
    {
        Jointfdmsg.Joint_fdpst[i]=Mt_fdpst[i]/18000.0*pi;
        Jointfdmsg.Joint_fdspd[i]=Mt_fdspd[i]/18000.0*pi;
        Jointfdmsg.Joint_fdctr[i]=Mt_fdcrt[i]/1.0;
    }
	//ROS_INFO("feedback [%d,%d,%d,%d]", Mt_fdpst[0],Mt_fdpst[1],Mt_fdpst[2],Mt_fdpst[3]);

    Joint_fdpub.publish(Jointfdmsg);

	if(test!=0)
		feedbackStart=1;
	if(sysstatus==0xff)
		sysstatus=0;
}

void MpumsgCB(const bmirobot_msg::Robot_mpu::ConstPtr& msg)
{
    for(int i=0;i<8;i++)
    {
        mpuAx[0+8*i]=(msg->mpu_Ax[i]);
        mpuAy[1+8*i]=(msg->mpu_Ay[i]);
        mpuAz[2+8*i]=(msg->mpu_Az[i]);
        mpuRx[3+8*i]=(msg->mpu_Rx[i]);
        mpuRy[4+8*i]=(msg->mpu_Ry[i]);
        mpuRz[5+8*i]=(msg->mpu_Rz[i]);
    }
//    ROS_INFO("new motor command: [%x,%x,%x]", msg->mtmode[0],msg->mtmode[1],msg->mtmode[2]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bmirobot_hw_V4");

	ros::NodeHandle n("bmirobot");
	ros::Duration period(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	//ros::Rate loop_rate(1000);

	MT_ctrpub = n.advertise<bmirobot_msg::Robot_ctr>("MT_ctr", 1000);
	Joint_fdpub = n.advertise<bmirobot_msg::Robot_jointfd>("MT_Jointfd", 1000);
	ros::Subscriber MT_fdctrsub = n.subscribe("MT_fdctr", 10,FdctrmsgCB);
	//ros::Subscriber MT_mpusub = n.subscribe("MT_fdmpu", 100, MpumsgCB);

	BMIRobot robot;
	controller_manager::ControllerManager cm(&robot,n);
	usleep(300);
 	std::cout << argc << argv[1]<<'\n';

  	string saveflag;
  	n.getParam("/savedata", saveflag);

  	int count;

	time_t t = time(0);
	char tmp[100];

	struct tm *info;
	info = localtime(&t);
	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/%d-%d-%d-%d-%d-%d-mtctr.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/new-mtctr.csv");
	std::cout<<tmp<<endl;
   	ctrout.open(tmp, std::ofstream::out | std::ofstream::binary);

	if(saveflag=="true")
		sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/%d-%d-%d-%d-%d-%d-pid.csv", 1900+info->tm_year, 1+info->tm_mon, info->tm_mday,info->tm_hour,info->tm_min,info->tm_sec);
	else
		sprintf(tmp,"/home/bmi/Dropbox/ws_moveit/new-pid.csv");
	std::cout<<tmp<<endl;
   	pidout.open(tmp, std::ofstream::out | std::ofstream::binary);




	  while (ros::ok())
	  {
		//ros::spinOnce();
		switch(sysstatus)
		{
			case 0:
				MTctrmsg.mtID[0]=0;
				MTctrmsg.mtID[1]=0;
				MTctrmsg.mtID[2]=0;
				MTctrmsg.mtID[3]=1;
				MTctrmsg.mtID[4]=2;
				MTctrmsg.mtID[5]=4;
				MTctrmsg.mtID[6]=3;
				MTctrmsg.mtID[7]=0;

			    MTctrmsg.mtmode[0]=M1_ctr;
			    MTctrmsg.mtmode[1]=M2_ctr;
		        MTctrmsg.mtmode[2]=M3_ctr;
		        MTctrmsg.mtmode[3]=M4_ctr;
				MTctrmsg.mtmode[4]=M5_ctr;
				MTctrmsg.mtmode[5]=M6_ctr;
				MTctrmsg.mtmode[6]=M7_ctr;
				MTctrmsg.mtmode[7]=M8_ctr;

				MT_ctrpub.publish(MTctrmsg);
				ROS_ERROR("change motor mode");	
				if(feedbackStart==1)
				//if((Mt_mode[0]&0xff)==M1_ctr)
				//if((Mt_mode[1]&0xff)==M2_ctr)
				//if((Mt_mode[2]&0xff)==M3_ctr)
				if((Mt_mode[3]&0xff)==M4_ctr)
				if((Mt_mode[4]&0xff)==M5_ctr)
				//if((Mt_mode[5]&0xff)==M6_ctr)
				//if((Mt_mode[6]&0xff)==M7_ctr)
				//if((Mt_mode[7]&0xff)==M8_ctr)
				{
					sysstatus=1;
				}
			break;

			case 1:
				MTctrmsg.mtmode[0]=M1_ctr+0x30;
			    MTctrmsg.mtmode[1]=M2_ctr+0x30;
		        MTctrmsg.mtmode[2]=M3_ctr+0x30;
		        MTctrmsg.mtmode[3]=M4_ctr+0x30;
				MTctrmsg.mtmode[4]=M5_ctr+0x30;
				MTctrmsg.mtmode[5]=M6_ctr+0x30;
				MTctrmsg.mtmode[6]=M7_ctr+0x30;
				MTctrmsg.mtmode[7]=M8_ctr+0x30;
				ROS_ERROR("start motor");
				MT_ctrpub.publish(MTctrmsg);		
				//if(Mt_mode[4]==Motor_mode+Motor_status+0x30)
				sysstatus=2;

			break;
	
			case 2:
				usleep(1000);
				//float init_x[2]={0,0};
			    //float init_p[2][2] = {{10e-6,0}, {0,10e-6}};
				//kalman2_init(&kalman[0], init_x, init_p);

				sysstatus=3;
			break;

			case 3:
				robot.read();
				cm.update(robot.get_time(), robot.get_period());
				robot.write();
			break;
		}
		//count=count%10000;

        //ROS_INFO("motor feedback:,%d,%d,%d,%d", Mt_fdcrt[0],Mt_fdtq[0],MTctrmsg.mttq[0], Mt_fdspd[0]);
        //ros::spinOnce();
        //loop_rate.sleep();
  //	MT_ctrpub.publish(MTctrmsg);
		//ros::spinOnce();
    	usleep(500);
		//loop_rate.sleep();
  }
  //spinner.stop();
}

