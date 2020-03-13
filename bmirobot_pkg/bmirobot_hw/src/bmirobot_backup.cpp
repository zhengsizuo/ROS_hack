#include <bmirobot_hw/bmirobot.h>
#include <serial/serial.h>
#include <vector>
#include <iomanip> 
#include <ros/console.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define RAD2DEG(x) (x/3.1415926*180)
#define DEG2RAD(x) (x/180.0*3.1415926)
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
int direction[8]={-1,-1,-1,-1,-1,1,1,-1};
int home_pos[8]={0,0,0,0,0,0,0,518};
double gears[8]={1,1,1,1,0,0,0,0};
using namespace std;

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
  size_t bytes_wrote = my_serial.write("US");
  bytes_wrote = my_serial.write("US");

  }
  void BMIRobot::read() 
{
  for(int i=0;i < JOINT_NUM+HAND_JOINT_NUM;i++)
  {
     pos[i] = cmd[i];
     // vel[i] = 1;
     // eff[i] = 1;
  }
}

void BMIRobot::write(double* a) 
{

    Eigen::Matrix4d ICSCoeff,ICSCoeff_W;
    ICSCoeff<< 	0.5,	0.5,	0,	0,
	        0.5,	-0.5,	0,	0,
		0,	0,	0.5,	0.5,
		0,	0,	0.5,	-0.5;

    ICSCoeff_W<<17,	17,	0,	0,
		0.5,	-0.5,	0,	0,
		0,	0,	1,	0,
		0,	0,	0,	1;

    ICSCoeff_W(0,0) /= 48.0;
    ICSCoeff_W(0,1) /= 48.0;
    Eigen::Vector4d J,M,J_W,M_W,midAngle;//J = ICSCoeff*M

    midAngle << 7500,7500,7500,7500;

    
    

    for(int i=0;i < JOINT_NUM+1;i++)
    {
       //cmd[i] = a[i];
      ROS_INFO("%f ", cmd[i]);
      if(i < 4)
      {
		J(i) = cmd[i]*direction[i];
      }
	else
	{
 		J_W(i-4) = cmd[i]*direction[i];
	}
       //printf("%f ",cmd[i]);

    }
    //J_W(3) = 0;
    cout << '\n';
    unsigned char motroCmd[JOINT_NUM*2+2];
    M = ICSCoeff.inverse()*J;
    M_W = ICSCoeff_W.inverse()*J_W;
    for(int i=0;i < JOINT_NUM+1;i++)
    {
      //convert to the robot angle
      //cmd[i] = cmd[i]*direction[i] + home_pos[i];

      	cout << transFormAangle(i<4?M(i):M_W(i-4),motroCmd[i*2],motroCmd[i*2+1],home_pos[i]) << " ";
      	printf("0x%X_0x%X ",motroCmd[i*2], motroCmd[i*2+1]); 
     //cout << std::hex << motroCmd[i*2] ;
      //cout << std::hex << motroCmd[i*2+1] << " ";
    }
    cout << std::endl;
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

    printf("\n");
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
