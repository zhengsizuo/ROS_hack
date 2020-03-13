#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Vector3.h>
#include <fstream>
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65


double joint_states[9];

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0;i < 9;i++)
		joint_states[i] = msg->position[i];
//ROS_INFO("receive joint states...");
//firstJointF = 1;
        
}
class TeleopPR2Keyboard
{
  private:
  geometry_msgs::PoseStamped cmd;

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;

  public:
  void init()
  {
    //header - this is impt
    cmd.header.frame_id = "/odom_combined";

    //Clear out our cmd - these values are roundabout initials
    cmd.pose.position.x=0.4;
    cmd.pose.position.y=0;
    cmd.pose.position.z=-0.29;
    cmd.pose.orientation.x=0;
    cmd.pose.orientation.y=0;
    cmd.pose.orientation.z=0;
    cmd.pose.orientation.w=1;

    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("bmirobot/head_controller/command", 1);

    ros::NodeHandle n_private("~");
  }

  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

std::vector<pcl_msgs::ModelCoefficients> sphere_coefs;
std::vector< std::vector<double> > joint_array;
int saveF = 0;
std::vector<tf::Vector3> trans;
void objectFindCallback(const pcl_msgs::ModelCoefficientsPtr& msg)
{
//perror("save camera:");
pcl_msgs::ModelCoefficients sphere_coef;
    //read the modelcoefficient
    sphere_coef = *msg;
    if(saveF == 1)
{
    	sphere_coefs.push_back(sphere_coef);
        saveF = 0;
perror("save camera:");
}
    
   
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_record");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("reco_sphere_coffi", 1, objectFindCallback);
  ros::Subscriber jointStatesSubscriber = nh.subscribe("/bmirobot/joint_states", 1,jointStatesCallback);
  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' to forward/back");
  puts("Use 'AD' to left/right");
  puts("Use 'QE' to up/down");

  for(;;)
  {
ros::Duration(1.0).sleep();
    // get the next event from the keyboard
ros::spinOnce();
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    float step = 0.02;
    switch(c)
    {

    case KEYCODE_S:
{
        if(sphere_coefs.size()==0||trans.size()==0||sphere_coefs.size()!=trans.size())
	    {
	      perror("error():");
	      exit(-1);
	    }
	std::ofstream myfile,myfile2;
	myfile.open ("/home/bmi/Documents/ws_moveit/src/bmirobot_pkg/bmirobot_calibration/record.txt");
        myfile2.open ("/home/bmi/Documents/ws_moveit/src/bmirobot_pkg/bmirobot_calibration/jointRecord.txt");
	myfile << sphere_coefs.size() << '\n';
	myfile2 << sphere_coefs.size() << '\n';

	for(int i = 0;i < sphere_coefs.size();i++)
	{
	     myfile << sphere_coefs[i].values[0] << " "<< sphere_coefs[i].values[1] << " "<< sphere_coefs[i].values[2] << "\n";
	     myfile << trans[i][0] << " "<< trans[i][1] << " "<< trans[i][2] << "\n";
		for(int j = 2;j < 9;j++)
			myfile2 << joint_array[i][j] << " ";
		for(int j = 0;j < 2;j++)
			myfile2 << joint_array[i][j] << " ";
		myfile2 << "\n";
	}
	//myfile << "Writing this to a file.\n";
	myfile2.close();
	myfile.close();
      perror("save file:");
}
      break;
    case KEYCODE_A:
      saveF = 1;
      //save transform
	  tf::TransformListener listener;
	  //tf::Transform transform;
	  ros::Duration(1.0).sleep();
	  tf::StampedTransform transform;
	  try{

		    listener.lookupTransform("/base_link", "/link_calibration",  
				             ros::Time(0), transform);


		tf::Vector3 pin;
		   pin[0] = 0;
		   pin[1] = 0;
		   pin[2] = 0;
		   tf::Vector3 pout;
		pout = transform(pin);
		trans.push_back(pout);

perror("save robot:");
		}
	  catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	    ros::Duration(1.0).sleep();
	  }
	std::vector<double> temp;
         for(int i = 0;i< 9;i++)
	{

	    temp.push_back(joint_states[i]);	
	}
        joint_array.push_back(temp);
      
      break;

    }



  }
}
