#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include "bmi_gui/displayMode.h"

using namespace cv;
using namespace std;

void overlayImage(Mat* src, Mat* overlay, const Point& location)
{
    for (int y = max(location.y, 0); y < src->rows; ++y)
    {
        int fY = y - location.y;

        if (fY >= overlay->rows)
            break;

        for (int x = max(location.x, 0); x < src->cols; ++x)
        {
            int fX = x - location.x;

            if (fX >= overlay->cols)
                break;

            double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

            for (int c = 0; opacity > 0 && c < src->channels(); ++c)
            {
                unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
                unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
                src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }
}

class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
  cv::Mat arrowImg;
  cv::Mat confirmImg;
  int curMode;
  cv::Point3d target;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = "/camera/rgb/image_raw";//nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
    arrowImg = cv::imread("/home/bmi/Documents/ws_moveit/src/bmirobot_pkg/bmi_gui/src/arrow.png", IMREAD_UNCHANGED);
confirmImg = cv::imread("/home/bmi/Documents/ws_moveit/src/bmirobot_pkg/bmi_gui/src/confirm.png", IMREAD_UNCHANGED);
curMode = 0;


  }
  bool displayMode(bmi_gui::displayMode::Request  &req,
       bmi_gui::displayMode::Response &res)
     {
     //res.sum = req.a + req.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
      //res.status = runStatus;
      curMode = req.mode;
	target.x = req.x;
	target.y = req.y;
	target.z = req.z;	

      return true;
    }
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
      cv::Point2d uv;
    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
      tf::StampedTransform transform;
      try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     acquisition_time, transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());

      uv = cam_model_.project3dToPixel(pt_cv);

      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvSize text_size;
      int baseline;
      cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                               uv.y - RADIUS - baseline - 3);
//ROS_WARN("[draw_frames] TF exception:%s", frame_id.c_str());
    cv:putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0));
    }
      static const int RADIUS = 3;
      CvSize text_size;
      int baseline;
      //cv::Point2d uv;

string displayStr;
uv = cam_model_.project3dToPixel(target);
      switch(curMode) {
	case 0://ready wait to start
displayStr = "The robot is ready and wait to start";
	break;	
	case 1://move the arm to object
displayStr = "Please move the arm to object";
    overlayImage(&image,&arrowImg,Point(0,0));
	break;
	case 2://confirm the object
displayStr = "Please confirm the object";
    overlayImage(&image,&confirmImg,Point(uv.x-126,uv.y-126));
	break;
	case 3://auto grasping
displayStr = "Please wait for auto grasping";
	break;
	case 4://move the arm to place
displayStr = "Please move the arm to place the object";
    overlayImage(&image,&arrowImg,Point(0,0));
	break;
	case 5://confirm the place
displayStr = "Please confirm the place";
    overlayImage(&image,&confirmImg,Point(uv.x-126,uv.y-126));
	break;
	case 6://auto placing
displayStr = "Please wait for auto placing";
	break;
}
cvGetTextSize(displayStr.c_str(), &font_, &text_size, &baseline);
CvPoint origin = cvPoint(image.cols/2 - text_size.width / 2,
                               image.rows-baseline -20);
cv::rectangle(image,Point(0,image.rows-baseline -text_size.height-40),Point(image.cols,image.rows-baseline),CV_RGB(255,255,255),CV_FILLED);
//cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
putText(image, displayStr.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0),2);
    pub_.publish(input_bridge->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
           ros::NodeHandle n;
ros::ServiceServer service = n.advertiseService("change_display_mode", &FrameDrawer::displayMode,&drawer);
  ros::spin();
}
