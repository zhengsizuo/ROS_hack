#include <bmirobot_hw/bmirobot.h>
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>
#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
int direction[8]={-1,-1,-1,-1,-1,-1,-1,-1};
//float home_pos[9]={0.07,0.05,-0.21,0.03,-0.24,0.47,0.14,-0.27,-0.27};
//float home_pos[9]={0.01,-0.28,0.35,-0.29,-0.08,-0.19,0.13,0.29,0.29};
//float home_pos[9]={0,0,0,0,0,0,0,0,0};


float home_pos[9]={0,0,-0.11,-0.14,0.84,0.14,0.06,0.42,0.42};
//float home_pos[9]={0,-0.05,-0.11,-0.20,0.84,0.0,-0.04,0.42,0.42};

double gears[8]={1,1,1,1,0,0,0,0};
float joint_limition[9][2];
Eigen::Matrix4d ICSCoeff,ICSCoeff_W;
Eigen::Vector4d J,M,J_W,M_W,midAngle;//J = ICSCoeff*M
using namespace std;
#define USE_ODOMETER 0

int enumerate_ports(string portNum)
{
  vector<serial::PortInfo> devices_found = serial::list_ports();

  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while( iter != devices_found.end() )
  {
    serial::PortInfo device = *iter++;
    if(device.port==portNum)
    {
      return(1);
    }
    //printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
    // device.hardware_id.c_str() );

  }
  printf( "\n!!!!!!!!!!!!!not found the serial port %s\n",portNum.c_str());
  return(0);
}
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
  int step = 0;
  readDataICS(step,H,L);

  double maxAngle = DEG2RAD(270);
  int maxStep = 11500-3500;
  double a = (step - 3500 - homepos)*maxAngle/maxStep-maxAngle/2;
  //int step =  int((a+maxAngle/2)*maxStep/maxAngle)+3500+homepos;
  //if(step < 3500)
  //   step = 3500;
  //if(step > 11500)
  //   step = 11500;
  //int step =  int((a)*maxStep/maxAngle);
  //formatDataICS(step,H,L);
  return a;
}
/*
int transFormAangle(double a,unsigned char& H,unsigned char& L)
{
  double maxAngle = DEG2RAD(270);
  int maxStep = 11500-3500;
  //int step =  int((a+maxAngle/2)*maxStep/maxAngle)-3500;
  int step =  int((a)*maxStep/maxAngle);
  formatData(step,H,L);
  return step;
}
*/
char handJN[][13]={"hand_joint_1","hand_joint_4"};
BMIRobot::~BMIRobot() 
{
  my_serial.write("UR");
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
std::cout<< "parse urdf file ok";
 boost::shared_ptr<const urdf::Link> link = model.getLink("base_link");
std::cout<< link->child_joints[0]->limits->lower << "   ";
joint_limition[0][0] = link->child_joints[0]->limits->lower;
joint_limition[0][1] = link->child_joints[0]->limits->upper;

  for(int i = 0;i<7;i++)
  {
    link = link->child_links[0];
	int index = 0;
    if(i == 6)
	index = 1;

    	std::cout<< link->child_joints[index]->limits->lower << "   ";
	joint_limition[i+1][0] = link->child_joints[index]->limits->lower;
	joint_limition[i+1][1] = link->child_joints[index]->limits->upper;
  }
  joint_limition[8][0] = -1*joint_limition[7][1];
  joint_limition[8][1] = -1*joint_limition[7][0];
  //exit(0);
  //joint_limition
  //
    ICSCoeff<< 	0.5,	0.5,	0,	0,
	        0.5,	-0.5,	0,	0,
		0,	0,	0.5,	0.5,
		0,	0,	0.5,	-0.5;

    ICSCoeff_W<<1,	0,	0,	0,
		0,	0.5,	-0.5,	0,
		0,	0.5,	0.5,	0,
		0,	0,	0,	1;

    float joint12 = 28/38.0;
    ICSCoeff(0,0) *= joint12;
    ICSCoeff(0,1) *= joint12;
    //ICSCoeff(1,0) *= joint12;
    //ICSCoeff(1,1) *= joint12;
    
    ICSCoeff(2,2) *= joint12;
    ICSCoeff(2,3) *= joint12;
    //ICSCoeff(3,2) *= joint12;
    //ICSCoeff(3,3) *= joint12;


    float joint6 = 18/14.0;
    ICSCoeff_W(1,1) *= joint6;
    ICSCoeff_W(1,2) *= joint6;

    float joint7 = 18/14.0;
    //ICSCoeff_W(2,1) *= joint67;
    //ICSCoeff_W(2,2) *= joint67;
    ICSCoeff_W(2,1) *= joint7;
    ICSCoeff_W(2,2) *= joint7;

    

    midAngle << 7500,7500,7500,7500;

  
    // connect and register the joint state interface
  for(int i=0;i < JOINT_NUM+HAND_JOINT_NUM;i++)
  {
     pos[i] = cmd[i] =0;
     // vel[i] = 1;
     // eff[i] = 1;
  }
   char jointName[255];
   char baseName[]="joint";
   portNum = "/dev/ttyUSB0";
   //int i = 0;
   for(int i =0;i < JOINT_NUM;i++)
  {
     sprintf(jointName,"%s%i",baseName,i+1);
     printf("%s",jointName);
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
     // connect and register the joint position interface
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


  int BMIRobot::initrobot()
  {
    int res = enumerate_ports(portNum);
    if(res == 0)
      return 0;
    my_serial.setPort(portNum);
    my_serial.setBaudrate(115200);
    my_serial.setTimeout(serial::Timeout::simpleTimeout(1000));
    my_serial.open();
    cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
  //size_t bytes_wrote = my_serial.write("FR");
  for(int i=0;i< 5;i++)
  {
  	my_serial.write("FB");

     usleep(5000);
     unsigned char motorP[16];
     unsigned char head[2];
     my_serial.read(head,2); 
     my_serial.read(motorP,16);
     read1();
 
  } //bytes_wrote = my_serial.write("US");

 }
void BMIRobot::read1()
{
my_serial.write("FB");

  usleep(5000);
  unsigned char motorP[16];
  unsigned char head[2];
   my_serial.read(head,2); 
  my_serial.read(motorP,16);

  for(int i=0;i < JOINT_NUM+1;i++)
  {
      if(i < 4)
      {
		M(i) = readAangle(motorP[i*2],motorP[i*2+1]);
		ROS_INFO("%f ", M(i) );
      }
      else
      {
 		M_W(i-4) = readAangle(motorP[i*2],motorP[i*2+1]);
       		ROS_INFO("%f ", M_W(i-4) );
      }
     //pos[i] = cmd[i];
     //vel[i] = 1;
     //eff[i] = 1;
  }
  ROS_INFO("----------------------");
  J = ICSCoeff*M;
  J_W = ICSCoeff_W*M_W;
  for(int i=0;i < JOINT_NUM+1;i++)
  {
           if(i < 4)
      {
		pos[i] = J(i)*direction[i]-home_pos[i];
      }
	else
	{
 		pos[i] = J_W(i-4)*direction[i]-home_pos[i];
	}
     //pos[i] = cmd[i];
     // vel[i] = 1;
     // eff[i] = 1;
  }
  //pos[JOINT_NUM] = pos[JOINT_NUM];
  pos[JOINT_NUM+1] = -1*pos[JOINT_NUM];
}


void BMIRobot::read() 
{
#if USE_ODOMETER==0
  for(int i=0;i < JOINT_NUM+2;i++)
  {

     pos[i] = cmd[i];
     // vel[i] = 1;
     // eff[i] = 1;
  }
#else

  

#endif
  
}

void BMIRobot::write(double* a,bool useF) 
{
	///
	float step = 0.025;
	/*
	for(int i=0;i < JOINT_NUM+2;i++)
	    {
	      
	      if(cmd[i] - pos[i] > 0 && cmd[i] - pos[i] > step)
	      {
		cmd[i] = pos[i] + step;
	      }
	      else if(cmd[i] - pos[i] < 0 && cmd[i] - pos[i] < step)
	      {
		cmd[i] = pos[i] - step;
	      }
	    }
	*/
	if(useF == true)
	{
	    for(int i=0;i < JOINT_NUM+1;i++)
	    {
	       cmd[i] = a[i];
	      //ROS_INFO("%f ", cmd[i]);
	    }
	}
	float maxV = 0;
	int maxI=0;
	float diff[JOINT_NUM+2];

	for(int i=0;i < JOINT_NUM+2;i++)
    {
        diff[i] = cmd[i] - pos[i];
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
	      cmd[i] = pos[i]+diff[i]*step/maxV;
	      //diff[i] = cmd[i] - pos[i];
	    }
	}
	////
	for(int i=0;i < JOINT_NUM+2;i++)
    {
		if(cmd[i]< joint_limition[i][0])
			cmd[i]= joint_limition[i][0];
		if(cmd[i]> joint_limition[i][1])
			cmd[i]= joint_limition[i][1];
    }
	////
	    for(int i=0;i < JOINT_NUM+1;i++)
	    {
	       //cmd[i] = a[i];
	      //ROS_INFO("%f ", cmd[i]);
	      if(i < 4)
	      {
			J(i) = (cmd[i]+home_pos[i])*direction[i];
	      }
		else
		{
	 		J_W(i-4) = (cmd[i]+home_pos[i])*direction[i];
	//printf("%f ",J_W(i-4));
		}
	       

	    }
	    //J_W(1) = 0;
	    //J_W(2) = 0;
	    //cout << '\n';
	    unsigned char motroCmd[JOINT_NUM*2+2];
	    M = ICSCoeff.inverse()*J;
	    M_W = ICSCoeff_W.inverse()*J_W;
	//cout << ICSCoeff_W;
	    //cout << M_W;
	    for(int i=0;i < JOINT_NUM+1;i++)
	    {
	      //convert to the robot angle
	      //cmd[i] = cmd[i]*direction[i] + home_pos[i];

	      	cout << transFormAangle(i<4?M(i):M_W(i-4),motroCmd[i*2],motroCmd[i*2+1],0) << " ";
	      	//printf("0x%X_0x%X ",motroCmd[i*2], motroCmd[i*2+1]); 
	     //cout << std::hex << motroCmd[i*2] ;
	      //cout << std::hex << motroCmd[i*2+1] << " ";
	    }
	    //cout << std::endl;
	    //transFormAangle(0,motroCmd[JOINT_NUM*2],motroCmd[JOINT_NUM*2+1]);
	    cout << my_serial.write("PP") << '\n';
	    cout << my_serial.write(motroCmd,8*2)<<'\n';
	    usleep(50000);
	    //my_serial.write("SP");
	    //my_serial.write(motroCmd+4*2,4*2);   
	/*
	    for(int i=0;i < JOINT_NUM;i++)
	    {
	      //convert to the robot angle
	      cmd[i] = cmd[i]*direction[i] + home_pos[i];
	      
	      cout << transFormAangle(cmd[i],motroCmd[i*2],motroCmd[i*2+1]) << " ";
	      printf("0x%X_0x%X ",motroCmd[i*2], motroCmd[i*2+1]); 
	      //cout << std::hex << motroCmd[i*2] ;
	      //cout << std::hex << motroCmd[i*2+1] << " ";
	    }
	    cout << std::endl;
	    transFormAangle(0,motroCmd[JOINT_NUM*2],motroCmd[JOINT_NUM*2+1]);
	    cout << my_serial.write("PA") << '\n';
	    cout << my_serial.write(motroCmd,4*2)<<'\n';
	    usleep(50000);
	    my_serial.write("SA");
	    my_serial.write(motroCmd+4*2,4*2);    
	*/

	   // printf("\n");
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
