#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <std_msgs/String.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher say_pub;
public:
  ImageConverter()
    : it_(nh_)
  {	
    image_sub_ = it_.subscribe("thermal/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("image_converter/output_video", 1);
    say_pub = nh_.advertise<std_msgs::String>("/sayings", 1000);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    unsigned char* marray = cv_ptr->image.data;
    int brek = 0;
    int index = -1;
    int i_index = -1;
    int j_index = -1;
    for (int y = 0; y < cv_ptr->image.rows; y++) {
      if(brek == 1)
        break;
      for(int x = 0; x < cv_ptr->image.cols; x++) {
        uchar blue = ((uchar*)(cv_ptr->image.data + cv_ptr->image.step*y))[x*3];
        uchar green = ((uchar*)(cv_ptr->image.data + cv_ptr->image.step*y))[x*3+1];
        uchar red = ((uchar*)(cv_ptr->image.data + cv_ptr->image.step*y))[x*3+2];
        if(blue == 255){
          std::string robot_namespace;
          nh_.getParam("robot_namespace", robot_namespace);
          std_msgs::String payam;
          payam.data = "Alive victim detected by " + robot_namespace;
          say_pub.publish(payam);
          brek = 1;
          break;
        }
      }
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

