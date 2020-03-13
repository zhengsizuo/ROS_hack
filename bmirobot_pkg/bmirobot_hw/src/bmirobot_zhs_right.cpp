#include <bmirobot_hw/bmirobot_zhs.h>
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


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


bmirobot_msg::Robot_ctr rMTctrmsg;
bmirobot_msg::Robot_jointfd rJointfdmsg;
ros::Publisher rJoint_fdpub;
ros::Publisher rMT_ctrpub;

int32_t rfeedbackStart=0;
int32_t initcmd=0;


float home_pos[9]={0.04, -0.14, 0, -0.07, 0.3 , 0, 0, 0, 0};
//float home_pos[9]={0.00, -0.00, 0, -0.00, 0.5 , 0, 0, 0, 0};

int32_t rMt_mode[8];
int32_t rMt_fdpst[8];
int32_t rMt_fdspd[8];
int32_t rMt_fdcrt[8];
int32_t rMt_fdpwm[8];
int32_t rMt_fdtq[8];
int32_t rMt_abecd[8];
int32_t rMt_rlecd[8];

double rightcmd_updata[9]={0};

float rjoint_limition[9][2];
Eigen::Matrix<double, 8, 8> rM2J;
Eigen::Matrix<double, 8, 1> rJ_vect, rM_vect, rM_cmd;
double rinitpos[9]={0};
double rinitstep[9]={0};

using namespace std;
ofstream rfileout;


double savehandmotor(double goalpst, double fdpst,double speed, int crt)
{
	static double errupdate=0;
	double err=(goalpst-fdpst);
	double differr=errupdate-err;
	 errupdate=err;
	
	double torque=0;
	
	torque= err*6000.0+speed*-500.0;

	if(torque>2000)
		torque=2000;
	if(torque<-2000)	
		torque=-2000;

	ROS_INFO(" right save:,%12f,%12f,%12f,%12f",goalpst,fdpst,speed,torque);

	return (torque);


}

static double integalErr[8]={0},Err[8]={8},LastErr[8]={0}, dErr[8]={0};

static float pstP[8]={0.1,    	0.2,   	0.2,   	0.2,   	0.2,   	0.2,    0,	0};
static float pstI[8]={0.0005,  	0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0,	0};
static float pstD[8]={0.05,    	0.5,   	0.5,   	0.5,   	0.5,   	0.5,    0,	0};

double motor_pstPID(int num,double goalpst,double position, double speed, double current){
    double pstu=0;
    LastErr[num] = Err[num];
    Err[num] = goalpst- position;
    integalErr[num] += Err[num];
    dErr[num] = Err[num]-LastErr[num];
    pstu = pstP[num]*Err[num]+ pstD[num]*(speed)+ pstI[num]*integalErr[num];

    //ROS_INFO("motor pstout:,%12f,%12f,%12f,%12f,%12f,%12f,%12f",pstu,goalpst,Err[num],position,speed,dErr[num],integalErr[num]);
    return (double)(pstu);
}


static float P[8]={2,    2,   2,   2,   2,   2,    0,0};
static float I[8]={0.005,  0.005, 0.005, 0.005, 0.005, 0.005, 0,0};
static float D[8]={0.5,    0.5,   0.5,   0.5,   0.5,   0.5,    0,0};


double motor_spdPID(int num,double goalpst,double position, double speed, double current){
    double spdu=0;
    LastErr[num] = Err[num];
    Err[num] = goalpst- position;
    integalErr[num] += Err[num];
    dErr[num] = Err[num]-LastErr[num];
    spdu = P[num]*Err[num]+ D[num]*(speed)+I[num]*integalErr[num];
    if(spdu>180.0/180*pi)
        spdu=180.0/180*pi;
    if(spdu<-180.0/180*pi)
        spdu=-180.0/180*pi;
    ROS_INFO("motor spdout:,%12f,%12f,%12f,%12f,%12f,%12f,%12f",spdu,goalpst,Err[num],position,speed,dErr[num],integalErr[num]);
    return (double)(spdu);
}


static double tqP[8]={1,  		1,   		1,			1200, 		1200,    	2000,   2000,      	0};
static double tqI[8]={0.001,    0.001,     	0.001,  	1,   		1,  		10,  	10,      0};
static double tqD[8]={4,  		4,   		4,			40, 		40,         10,    10,      	2};


static double tqu[8]={0};
static double ipErr[8]={0}, pErr[8]={0};
static double ivErr[8]={0}, vErr[8]={0}, lvErr[8]={0},dvErr[8]={0};



double motor_tqPID(int num,double goalpst,double position, double speed, double current)
{
    double spdu=0,tquTemp=0;
    double errtqu=0;
    double step=100;
    double tqbound=3000.0;

    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
    //dErr[num] = Err[num]-LastErr[num];
    tqu[num] = tqP[num]*pErr[num]+ tqD[num]*(speed)+tqI[num]*ipErr[num];
    ROS_INFO("motor torqe:%2d ,tqu:%12f, gpst:%12f, pst:%12f, spd:%12f, crt:%12f, pE:%10f, ipE:%14f",num,tqu[num],goalpst, position, speed, current, pErr[num], ipErr[num]);

   // ROS_INFO("motor torqe:,%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f, %14f, %14f",tqu[num], spdu, speed, current, pwmduty, pErr[num], vErr[num], dvErr[num], ivErr[num], ipErr[num]);
    return (double)(tqu[num]);
}



static double PWMpP=3;
static double PWMpI=0.00;
static double PWMpD=0.5;
static double PWMvP=0.1;
static double PWMvI=0.000;
static double PWMvD=0.3;

static double PWMu[8]={0};

double motor_ForwardcontrolPID(int num,double goalpst,double position, double speed, double current)
{
    double spdu=0,PWMTemp=0;
    double errPWMu=0;
    double step=1;
    double PWMbound=100.0;

    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
    spdu = PWMpP*pErr[num]+ PWMpD*speed +PWMpI*ipErr[num];
    if(spdu>20*pi)
        spdu=20*pi;
    if(spdu<-20*pi)
        spdu=-20*pi;

    lvErr[num] = vErr[num];
    vErr[num]  = spdu-speed;
    ivErr[num] += vErr[num];
    dvErr[num] = vErr[num]-lvErr[num];
   // tqu1[num]=tqu[num];
    PWMTemp = vErr[num]*PWMvP + dvErr[num]*PWMvD +ivErr[num]*PWMvI;
    if(PWMTemp>PWMbound)
        PWMTemp=PWMbound;
    if(PWMTemp<-PWMbound)
        PWMTemp=-PWMbound;

    errPWMu=PWMTemp-PWMu[num];
    if(errPWMu>step)
    {
        PWMu[num]=PWMu[num]+step;
    }else if(errPWMu<-step)
    {
        PWMu[num]=PWMu[num]-step;
    }else
    {
       PWMu[num]=PWMTemp;
    }
    ROS_INFO("motor PWM:,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f",PWMu[num],spdu,speed,pErr[num],vErr[num],dvErr[num],ivErr[num],ipErr[num],current);
    return (double)(PWMu[num]);
}

void init_right_robot()
{
	rM2J<<   	-1.0/6.0,  	-1.0/3.0,	1.0/6.0,   0,      0,      0,      0,      0,
				-1.0/3.0, 	 1.0/3.0,	1.0/3.0,   0,      0,      0,      0,      0,
				-0.5,  		 0.0,   	-0.5,		0,		0,      0,      0,      0,
				0,      	0,      	0,			0.5,   -0.5,   0,      0,      0,
				0,      	0,      	0,      	-0.5,	-0.5,   0,      0,      0,
				0,      	0,      	0,      	0,      0,      0.5,    -0.5,   0,
				0,      	0,      	0,      	0,      0,      -0.5,   -0.5,   0,
				0,      	0,      	0,      	0,      0,      0,      0,      1;
	
	ros::NodeHandle node;


	char tmp[100];
	sprintf(tmp,"/home/bmi/bmiproject/ws_pc/data/right-mtctr.csv");
	std::cout<<tmp<<endl;
   	rfileout.open(tmp, std::ofstream::out | std::ofstream::binary);
	
	node.getParam("/home_rjoint1", home_pos[0]);
	node.getParam("/home_rjoint2", home_pos[1]);
	node.getParam("/home_rjoint3", home_pos[2]);
	node.getParam("/home_rjoint4", home_pos[3]);
	node.getParam("/home_rjoint5", home_pos[4]);
	node.getParam("/home_rjoint6", home_pos[5]);
	node.getParam("/home_rjoint7", home_pos[6]);
	node.getParam("/home_rjoint8", home_pos[7]);
}


void BMIRobot::right_read()
{
	unsigned char motorP[16];
	unsigned char head[2];

	for(int i=0;i < JOINT_NUM+1;i++)
	{
		rM_vect(i)= rJointfdmsg.Joint_fdpst[i];
	}
	// ROS_INFO("finish read data----------------------");
	rJ_vect = rM2J*rM_vect;
	
	for(int i=0;i < JOINT_NUM+1;i++)
	{
		//pos[i] =left_cmd[i]+random()%100/1000.0;//; //
		right_pos[i]= rJ_vect(i)-home_pos[i];
		//pos[i]=leftcmd_updata[i];
	}
  
	right_pos[JOINT_NUM+1] = -1*right_pos[JOINT_NUM];
	//ROS_INFO("pos:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],pos[7],pos[8]);

	if(initcmd==0)
	{
		for(int i=0;i < JOINT_NUM+1;i++)
	    {
			rightcmd_updata[i]=rJ_vect(i)-home_pos[i];
		}
		initcmd=1;
	}
}

int32_t testcount=0;
int32_t tstep=0;
ros::Time init_time ;
ros::Time now_time ;
int32_t counttest=0;
void BMIRobot::right_write()
{
    double pstctr,speedctr,torquectr,PWMctr;

	float step = 0.02;

	float maxV = 0;
	int maxI=0;
	float diff[JOINT_NUM+2];
	double diftime;

	ROS_INFO("right_cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",right_cmd[0],right_cmd[1],right_cmd[2],right_cmd[3],right_cmd[4],right_cmd[5],right_cmd[6],right_cmd[7]);

 	for(int i=0;i < JOINT_NUM+2;i++)
    {
        diff[i] = right_cmd[i] - rightcmd_updata[i];
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
	      rightcmd_updata[i] = rightcmd_updata[i]+diff[i]/maxV*step;
	    }
    }else
	{	
	    for(int i=0;i < JOINT_NUM+2;i++)
	    {
			rightcmd_updata[i]=rightcmd_updata[i]+diff[i];
		}
	}


	for(int i=0;i < JOINT_NUM+2;i++)
    {
		if(rightcmd_updata[i]< rjoint_limition[i][0])
			rightcmd_updata[i]= rjoint_limition[i][0];
		if(rightcmd_updata[i]> rjoint_limition[i][1])
			rightcmd_updata[i]= rjoint_limition[i][1];
    }
	//ROS_INFO("rJ_limit:%f,%f,%f,%f,",rjoint_limition[7][0],rjoint_limition[7][1],rjoint_limition[8][0],rjoint_limition[8][1]);

    for(int i=0;i < JOINT_NUM+1;i++)
    {
        rJ_vect(i) = (rightcmd_updata[i]+home_pos[i]);
    }
	//ROS_INFO("rJ_vect:%f,%f,%f,%f,%f,%f,%f,%f",rJ_vect(0),rJ_vect(1),rJ_vect(2),rJ_vect(3),rJ_vect(4),rJ_vect(5),rJ_vect(6),rJ_vect(7));

    unsigned char motroCmd[JOINT_NUM*2+2];
    rM_vect= rM2J.inverse()*rJ_vect;
    rM_cmd =rM_vect;

   	ROS_INFO("rM_vect:%f,%f,%f,%f,%f,%f,%f,%f",rM_vect(0),rM_vect(1),rM_vect(2),rM_vect(3),rM_vect(4),rM_vect(5),rM_vect(6),rM_vect(7));

    for(int i=0;i < 8;i++)
    {
		switch(rMTctrmsg.mtmode[i]&0x0c)
		{
			case pst_mode:
					rMTctrmsg.mtpst[i]=rM_cmd(i)*(18000.0/pi);
					//pstctr=	motor_pstPID(i,rM_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
					//MTctrmsg.mtpst[i]=(int32_t)(pstctr/pi*18000.0);
				break;
			case spd_mode:
			       	//speedctr=motor_spdPID(i,rM_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
			        //MTctrmsg.mtspd[i]=(int32_t)(speedctr/pi*18000.0);
				break;
			case tq_mode:
       				torquectr = motor_tqPID(i,rM_vect(i),rJointfdmsg.Joint_fdpst[i],rJointfdmsg.Joint_fdspd[i],rJointfdmsg.Joint_fdctr[i]);
     				rMTctrmsg.mttq[i] =(int32_t) torquectr;
				break;
			case fw_mode:
					//PWMctr=motor_ForwardcontrolPID(i,rM_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Mt_fdcrt[i]*1.0);
					//MTctrmsg.mtpst[i] =(int32_t) (torquectr);
				break;
		}
		//ROS_INFO("mtmode:%x,   %x",MTctrmsg.mtmode[i],MTctrmsg.mtmode[i]&0xc0);
    }
	//rMTctrmsg.mtpst[7]=savehandmotor(rM_cmd(7),rJointfdmsg.Joint_fdpst[7],rJointfdmsg.Joint_fdspd[7],rJointfdmsg.Joint_fdctr[7]);
	//testcount++;
	//MTctrmsg.mttq[0]=testcount;
	//ROS_INFO("test:%d",testcount);
    rMT_ctrpub.publish(rMTctrmsg);

	ros::Time begin = ros::Time::now();
  	if(rfileout.is_open())
  	{
	   	rfileout <<begin<<",";
		for(int i=0;i<8;i++)
		{
			rfileout <<rM_vect(i)*(18000.0/pi)<<","<<rMt_fdpst[i]<<","<<rMt_fdspd[i]<<","<<rMt_fdcrt[i]<<",";	
		}
		rfileout<<endl; 
  	}
}

void  rFdctrmsgCB(const bmirobot_msg::Robot_fdctr::ConstPtr& msg)
{
	int32_t test=0;
    for(int i=0;i<8;i++)
    {
        rMt_mode[i]	=	msg->mt_mode[i];
        rMt_fdpst[i]= 	msg->mt_Cpst[i];
        rMt_fdspd[i]=(	msg->mt_Cspd[i]);
        rMt_fdcrt[i]=(	msg->mt_incrt[i]);
		test+=msg->mt_incrt[i];
    }
    for(int i=0;i<8;i++)
    {
        rJointfdmsg.Joint_fdpst[i]=rMt_fdpst[i]/18000.0*pi;
        rJointfdmsg.Joint_fdspd[i]=rMt_fdspd[i]/18000.0*pi;
        rJointfdmsg.Joint_fdctr[i]=rMt_fdcrt[i]/1.0;
    }
	//ROS_INFO("right feedback [%d,%d,%d,%d]", rMt_fdpst[0],rMt_fdpst[1],rMt_fdpst[2],rMt_fdpst[3]);

    rJoint_fdpub.publish(rJointfdmsg);

	if(test!=0)
		rfeedbackStart=1;
}


int BMIRobot::rightstep1(void)
{
	int rst=0;
	rMTctrmsg.mtID[0]=1;
	rMTctrmsg.mtID[1]=2;
	rMTctrmsg.mtID[2]=3;
	rMTctrmsg.mtID[3]=4;
	rMTctrmsg.mtID[4]=5;
	rMTctrmsg.mtID[5]=6;
	rMTctrmsg.mtID[6]=7;
	rMTctrmsg.mtID[7]=8;

	rMTctrmsg.mtmode[0]=M1_ctr;
	rMTctrmsg.mtmode[1]=M2_ctr;
	rMTctrmsg.mtmode[2]=M3_ctr;
	rMTctrmsg.mtmode[3]=M4_ctr;
	rMTctrmsg.mtmode[4]=M5_ctr;
	rMTctrmsg.mtmode[5]=M6_ctr;
	rMTctrmsg.mtmode[6]=M7_ctr;
	rMTctrmsg.mtmode[7]=M8_ctr;

	rMT_ctrpub.publish(rMTctrmsg);
	
	//ROS_ERROR("change right mode");	
	//cout<<"rfeed:"<<rfeedbackStart<<endl;
	//cout<<"motor mode:"<<((rMt_mode[0]&0x7f)==M1_ctr)<<endl;
	if(rfeedbackStart==1)
	if((rMt_mode[0]&0xff)==M1_ctr)
	if((rMt_mode[1]&0xff)==M2_ctr)
	if((rMt_mode[2]&0xff)==M3_ctr)
	if((rMt_mode[3]&0xff)==M4_ctr)
	if((rMt_mode[4]&0xff)==M5_ctr)
	if((rMt_mode[5]&0xff)==M6_ctr)
	if((rMt_mode[6]&0xff)==M7_ctr)
	//if((rMt_mode[7]&0x7f)==M8_ctr)
	{
		rst=1;
	}

	for(int i=0;i<8;i++)
	{
		rinitpos[i]=right_pos[i];
		right_cmd[i]=rinitpos[i];
		rinitstep[i]=rinitpos[i]*0.003;
	}
	return rst;
}



int BMIRobot::rightstep2(void)
{
	rMTctrmsg.mtmode[0]=M1_ctr+0x30;
	rMTctrmsg.mtmode[1]=M2_ctr+0x30;
	rMTctrmsg.mtmode[2]=M3_ctr+0x30;
	rMTctrmsg.mtmode[3]=M4_ctr+0x30;
	rMTctrmsg.mtmode[4]=M5_ctr+0x30;
	rMTctrmsg.mtmode[5]=M6_ctr+0x30;
	rMTctrmsg.mtmode[6]=M7_ctr+0x30;
	rMTctrmsg.mtmode[7]=M8_ctr+0x30;
	ROS_ERROR("start motor");
	rMT_ctrpub.publish(rMTctrmsg);
	return 1;
}

int BMIRobot::rightstep3(void)
{
	double total=0;
	for(int i=0;i<8;i++)
	{
		right_cmd[i]=rinitpos[i]-0.00001*rinitpos[i]; //初始化的速度
		total+=fabs(right_cmd[i]);
		rinitpos[i]=right_cmd[i];
	}

	ROS_ERROR("right to home,%f,%f,%f,%f,%f,%f,%f,%f,%f",total,rinitpos[0],rinitpos[1],rinitpos[2],rinitpos[3],rinitpos[4],rinitpos[5]);
	rMT_ctrpub.publish(rMTctrmsg);
	if(total<0.005)
		return 1;
	else
		return 0;	
}
