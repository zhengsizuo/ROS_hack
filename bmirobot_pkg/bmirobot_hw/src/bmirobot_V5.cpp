#include <bmirobot_hw/bmirobot_V4.h>
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_msg/Robot_jointfd.h>
#include <stdlib.h>
#include <controller_manager/controller_manager.h>

#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)
#define pi          3.1415926
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
#define USE_ODOMETER    0
#define Motor_mode      0x00   //motor mode: 0x00 position 0x04 speed 0x08 torque 0x0c forwardcontorl
#define Motor_status    0x00   //motor status (0x00 normal) (0x02 free) (0x03  hold)

int32_t sysstatus=0;

int direction[8]={1,1,1,1,1,1,1,1};

bmirobot_msg::Robot_ctr MTctrmsg;
bmirobot_msg::Robot_jointfd Jointfdmsg;
ros::Publisher Joint_fdpub;
ros::Publisher MT_ctrpub;

int32_t feedbackStart=0;
int32_t initcmd=0;


float home_pos[9]={-0,-0,-0,-0.0,0.00,0,0,0,0};

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



double gears[8]={1,1,1,1,0,0,0,0};
float joint_limition[9][2];
Eigen::Matrix<double, 8, 8> M2J;
Eigen::Matrix<double, 8, 1> J_vect, M_vect,M_cmd;

using namespace std;

void formatDataICS(int A,unsigned char& H,unsigned char& L)
{
  H = (A >> 7) & 0x7f;
  L = A  & 0x7f;
  //if(A < 0)
  //H = H | 0x80;
}
void readDataICS(int& A,unsigned char H,unsigned char L)
{
  //H = (A >> 7) & 0x7f;
  //L = A  & 0x7f;
  A = ((H << 8) + (A << 1))>>1; 
  //if(A < 0)
  //H = H | 0x80;
}
void formatData(int A,unsigned char& H,unsigned char& L)
{
  H = (A >> 8) & 0xff;
  L = A  & 0xff;
  if(A < 0)
    H = H | 0x80;
}

int transFormAangle(int a,unsigned char& H,unsigned char& L)
{
  int maxAngle = 270;
  int maxStep = 11500-3500;
  int step = int((a+maxAngle/2)*maxStep*1.0/maxAngle)-3500;
  formatData(step,H,L);
  return step;
}

int transFormAangle(double a,unsigned char& H,unsigned char& L,int homepos=0)
{
  double maxAngle = DEG2RAD(270);
  int maxStep = 11500-3500;
  int step =  int((a+maxAngle/2)*maxStep/maxAngle)+3500+homepos;
  if(step < 3500)
     step = 3500;
  if(step > 11500)
     step = 11500;
  //int step =  int((a)*maxStep/maxAngle);
  formatDataICS(step,H,L);
  return step;
}
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


static double tqP[8]={450,  450,   300,300, 200,     200,     0,      0};
static double tqI[8]={1,    1,     1.1,  1.1,   0.001,   0.001,   0,      0};
static double tqD[8]={200,  200,   80,80, 4,      4,      2,      2};


static double tqu[8]={0};
static double ipErr[8]={0}, pErr[8]={0};
static double ivErr[8]={0}, vErr[8]={0}, lvErr[8]={0},dvErr[8]={0};



double motor_tqPID(int num,double goalpst,double position, double speed, double current,double pwmduty)
{
    double spdu=0,tquTemp=0;
    double errtqu=0;
    double step=100;
    double tqbound=3000.0;

    pErr[num] = goalpst- position;
    ipErr[num] += pErr[num];
    //dErr[num] = Err[num]-LastErr[num];
    tqu[num] = tqP[num]*pErr[num]+ tqD[num]*(speed)+tqI[num]*ipErr[num];
    ROS_INFO("motor torqe:,tqu:%12f,pst:%12f,spd:%12f,crt:%12f,pwm:%12f,pE:%14f,ipE:%14f",tqu[num],position, speed, current, pwmduty*62.50, pErr[num], ipErr[num]);

   // ROS_INFO("motor torqe:,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%14f,%14f",tqu[num], spdu, speed, current, pwmduty, pErr[num], vErr[num], dvErr[num], ivErr[num], ipErr[num]);
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
  //

    M2J<<   -0.25,  -0.5,	0.25,   0,      0,      0,      0,      0,
            -0.25,  0.5,	0.25,   0,      0,      0,      0,      0,
            -0.5,  	0.0,   	-0.5,	0,		0,      0,      0,      0,
            0,      0,      0,		-0.5,	0.5,   	0,      0,      0,
            0,      0,      0,      -0.5,   -0.5,   0,      0,      0,
            0,      0,      0,      0,      0,      0.5,    -0.5,   0,
            0,      0,      0,      0,      0,      -0.5,   -0.5,   0,
            0,      0,      0,      0,      0,      0,      0,      1;
	
    //float joint12 = 1.0;
    //M2J(0,0) *= joint12;
    //M2J(0,1) *= joint12;
    //ICSCoeff(1,0) *= joint12;
    //ICSCoeff(1,1) *= joint12;
    
    //M2J(2,2) *= joint12;
    //M2J(2,3) *= joint12;
    //ICSCoeff(3,2) *= joint12;
    //ICSCoeff(3,3) *= joint12;
	
	/**
    float joint67 = 1.0;
    M2J(5,5) *= joint67;
    M2J(5,6) *= joint67;
    M2J(6,5) *= joint67;
    M2J(6,5) *= joint67;
  	**/
	
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
		// pos[i] =cmd[i]+random()%100/1000.0;//; //
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

}

void BMIRobot::write()
{
    double pstctr,speedctr,torquectr,PWMctr;

	float step = 0.02;

	float maxV = 0;
	int maxI=0;
	float diff[JOINT_NUM+2];

    //ROS_INFO("cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7]);

    for(int i=0;i < JOINT_NUM+2;i++)
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
	//ROS_INFO("diff:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",diff[0],diff[1],diff[2],diff[3],diff[4],diff[5],diff[6],diff[7]);
   // ROS_INFO("changed_cmd:%6f,%6f,%6f,%6f,%6f,%6f,%6f,%6f",cmd_updata[0],cmd_updata[1],cmd_updata[2],cmd_updata[3],cmd_updata[4],cmd_updata[5],cmd_updata[6],cmd_updata[7]);


	for(int i=0;i < JOINT_NUM+2;i++)
    {
		if(cmd_updata[i]< joint_limition[i][0])
			cmd_updata[i]= joint_limition[i][0];
		if(cmd_updata[i]> joint_limition[i][1])
			cmd_updata[i]= joint_limition[i][1];
    }
    for(int i=0;i < JOINT_NUM+1;i++)
    {
        J_vect(i) = (cmd_updata[i]+home_pos[i])*direction[i];
    }

    unsigned char motroCmd[JOINT_NUM*2+2];
    M_vect= M2J.inverse()*J_vect;
    M_cmd =M_vect*(18000.0/pi);
    ROS_INFO("M_vect:%f,%f,%f,%f,%f,%f,%f,%f",M_vect(0),M_vect(1),M_vect(2),M_vect(3),M_vect(4),M_vect(5),M_vect(6),M_vect(7));

    for(int i=0;i < 7+1;i++)
    {
		MTctrmsg.mtpst[i]=M_cmd(i);

		//pstctr=	motor_pstPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
		//MTctrmsg.mtpst[i]=(int32_t)(pstctr/pi*18000.0);

       //speedctr=motor_spdPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],0);
        //MTctrmsg.mtspd[i]=(int32_t)(speedctr/pi*18000.0);

       	//torquectr = motor_tqPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Mt_fdcrt[i]*1.0,Mt_fdpwm[i]/100.0);
     	//MTctrmsg.mttq[i] =(int32_t) torquectr;

        //PWMctr=motor_ForwardcontrolPID(i,M_vect(i),Jointfdmsg.Joint_fdpst[i],Jointfdmsg.Joint_fdspd[i],Mt_fdcrt[i]*1.0);
        //MTctrmsg.mtpst[i] =(int32_t) (torquectr);
        //cout << transFormAangle(i<4?M(i):M_W(i-4),motroCmd[i*2],motroCmd[i*2+1],0) << " ";
    }

	//MTctrmsg.mtpst[7]=savehandmotor(M_cmd(7),Jointfdmsg.Joint_fdpst[7],Mt_fdcrt[7]);

    MT_ctrpub.publish(MTctrmsg);
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
       //Mt_fdtq[i]=(msg->mt_Gtq[i]);
       // Mt_abecd[i]=(msg->mt_ecd[i]);
        //Mt_rlecd[i]=(msg->mt_ecdcnt[i]);
		test+=msg->mt_incrt[i];
    }
    for(int i=0;i<8;i++)
    {
        Jointfdmsg.Joint_fdpst[i]=Mt_fdpst[i]/18000.0*pi;
        Jointfdmsg.Joint_fdspd[i]=Mt_fdspd[i]/18000.0*pi;
        Jointfdmsg.Joint_fdctr[i]=Mt_fdcrt[i]/1.0;
    }
    Joint_fdpub.publish(Jointfdmsg);
	if(test!=0)
		feedbackStart=1;
	if(sysstatus==0xff)
		sysstatus=0;

   // ROS_INFO("motor feedback [%x,%x,%x,%x]", Mt_fdpst[0],Mt_fdspd[0],Mt_abecd[0],Mt_rlecd[0]);
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

  MT_ctrpub = n.advertise<bmirobot_msg::Robot_ctr>("MT_ctr", 1000);
  Joint_fdpub = n.advertise<bmirobot_msg::Robot_jointfd>("MT_Jointfd", 1000);
  ros::Subscriber MT_fdctrsub = n.subscribe("MT_fdctr", 10,FdctrmsgCB);
//  ros::Subscriber MT_mpusub = n.subscribe("MT_fdmpu", 1000, MpumsgCB);

  BMIRobot robot;
  controller_manager::ControllerManager cm(&robot,n);
  usleep(300);
  std::cout << argc << argv[1]<<'\n';

  int count;

  while (ros::ok())
  {
    switch(sysstatus)
    {
		case 0:
			MTctrmsg.mtID[0]=1;
			MTctrmsg.mtID[1]=2;
			MTctrmsg.mtID[2]=3;
			MTctrmsg.mtID[3]=5;
			MTctrmsg.mtID[4]=4;
			MTctrmsg.mtID[5]=6;
			MTctrmsg.mtID[6]=7;
			MTctrmsg.mtID[7]=8;

	        MTctrmsg.mtmode[0]=Motor_mode+Motor_status;
	        MTctrmsg.mtmode[1]=Motor_mode+Motor_status;
            MTctrmsg.mtmode[2]=Motor_mode+Motor_status;
            MTctrmsg.mtmode[3]=Motor_mode+Motor_status;
			MTctrmsg.mtmode[4]=Motor_mode+Motor_status;
			MTctrmsg.mtmode[5]=Motor_mode+Motor_status;
			MTctrmsg.mtmode[6]=Motor_mode+Motor_status;
			MTctrmsg.mtmode[7]=Motor_mode+Motor_status;

			MT_ctrpub.publish(MTctrmsg);
			ROS_ERROR("change motor mode");	
			if(feedbackStart==1)     
			if((Mt_mode[0]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[1]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[2]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[3]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[4]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[5]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[6]&0xff)==Motor_mode+Motor_status)
			if((Mt_mode[7]&0xff)==Motor_mode+Motor_status)
			{
				sysstatus=1;
			}
		break;

		case 1:
		    MTctrmsg.mtmode[0]=Motor_mode+Motor_status+0x30;
	        MTctrmsg.mtmode[1]=Motor_mode+Motor_status+0x30;
            MTctrmsg.mtmode[2]=Motor_mode+Motor_status+0x30;
            MTctrmsg.mtmode[3]=Motor_mode+Motor_status+0x30;
		    MTctrmsg.mtmode[4]=Motor_mode+Motor_status+0x30;
		    MTctrmsg.mtmode[5]=Motor_mode+Motor_status+0x30;
		    MTctrmsg.mtmode[6]=Motor_mode+Motor_status+0x30;
		    MTctrmsg.mtmode[7]=Motor_mode+Motor_status+0x30;
		    ROS_ERROR("start motor");
		    MT_ctrpub.publish(MTctrmsg);		
			//if(Mt_mode[4]==Motor_mode+Motor_status+0x30)
				sysstatus=2;
		break;

		case 2:
			robot.read();
			cm.update(robot.get_time(), robot.get_period());
			robot.write();			
		break;
    }
    /*int32_t spd5= motor_spdPID(0,Jointfdmsg.Joint_fdpst[5],Jointfdmsg.Joint_fdspd[5],Mt_fdcrt[5]);
	MTctrmsg.mtspd[5]=spd5;
    //    int32_t tq5= motor_tqPID(10000,Jointfdmsg.Joint_fdpst[5],Jointfdmsg.Joint_fdspd[5],Mt_fdcrt[5]);
    //    MTctrmsg.mttq[5]=tq5;
    */

	//MTctrmsg.mtpst[4]=30000*sin(count*2*3.1415/10000);
    //MTctrmsg.mttq[0]= -240;
	//MTctrmsg.mtspd[5]

        //count=count%10000;

        //ROS_INFO("motor feedback:,%d,%d,%d,%d", Mt_fdcrt[0],Mt_fdtq[0],MTctrmsg.mttq[0], Mt_fdspd[0]);
        //ros::spinOnce();
        //loop_rate.sleep();
  //	MT_ctrpub.publish(MTctrmsg);
    usleep(1000);
  }
  spinner.stop();
}

