#include <cstdlib>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <aut_exploration/Object.h>
#include <aut_exploration/Detections.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#define PI 3.14159265
//each laser beam hits an obstancle, so the distance is in a vector
  int imageRows = 0;
  int imageColumns = 0;
  std::vector<float> range;
  double dist = -1;
  double robot_yaw_angle = 0;
  double robot_position_angel=0;
  double robotX = 0;
  double robotY = 0;
  ros::Publisher dead_victPub, alive_victPub;
  std::vector<unsigned char> thermalImageArray;
  std::string robot_namespace;
  int laser_angle = 0;
  int image_pixel_width = 0;
  int laser_rayCount = 0;
  double camera_horizontal = 0;
//x and y of victim position
  struct victLocation {
    double x,y;
  };
//array of victims, so using this array assures us that a victim which is detected before, will not be detected next time
  std::vector<victLocation> victims;
//this function sends x and y of NEW victim to through a rostopic
   void send_message(victLocation vl,bool isAlive){
      ROS_INFO("Victim's message is publishing at: ");
      geometry_msgs::Point point;
      point.x = vl.x;
      point.y = vl.y;
      ROS_INFO("x = %f, y = %f ",vl.x, vl.y);
      if (isAlive)
        alive_victPub.publish(point);
      else
        dead_victPub.publish(point);
	  dist = -1;
    }
//this function checks all member of ranges array(from left of object to its //should_be_right middle ***) and returns minimum distance of them
    double computeDistance(int left,int right){
    // *** should change when laser changes
    double min = 18.0;
    ROS_INFO("leftrange %f, rightrange %f ",range[left],range[right]);
    ROS_INFO("leftbeam %d, rightbeam %d ",left,right);
    for (int i = left; i <= right; i++){
      //ROS_INFO("beam %d, range %f ",i,range[i]);
      if((range[i] <= min)&&(range[i] >= 1))
        min = range[i];
      }
    ROS_INFO("distance calculated!  \n");
    return min;
  }
//this function computes a detected victim's location so that we can check if it is detected before or not(now just for one robot ***)
  victLocation getVictimLocation(double vict_angle){
    if (dist == -1){
      ROS_INFO("Distance value is not allowed! it is -1 \n");
	}
    double victX = robotX + dist * cos(robot_yaw_angle -( vict_angle*PI / 180 ));
    double victY = robotY + dist * sin(robot_yaw_angle -( vict_angle*PI / 180 ));
    ROS_INFO("dist %f ryas %f  va %f \n",dist,robot_yaw_angle,vict_angle);
    victLocation vl;
    vl.x = victX;
    vl.y = victY;
    ROS_INFO("victim location is :  \n");
    return vl;
  }
//checks if victim which is detected now, is a new or one detected in the past
//copies laser ranges array(which there is distance to obstancle for each beam)
  void processLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
       range = scan->ranges;
  }
//if rail object detector published sth, it will be check human and being repetitious
  void processDetectionsCallback(const aut_exploration::Detections::ConstPtr& detect){
    ROS_INFO("detections called! \n");
     std::vector<aut_exploration::Object> obi = detect->objects;
    if(obi.size() > 0)
    {
      if (obi[0].label == "Human"){
        ROS_INFO("object found! \n");
        int right = obi[0].right_top_x - image_pixel_width / 2;
        double right_angle = atan2((double)right,(double)220.604700396)*((double)180/PI);
        //double right_angle = (double)((double)right / (double)image_pixel_width) * (double) camera_horizontal;
	int right_beam = (double)((double)((double)laser_rayCount / (double)laser_angle) * right_angle) + (double)(laser_rayCount / 2) ;
        //int rightBeam = (double)(((double)right / image_pixel_width) * laser_rayCount * (camera_horizontal/laser_angle)) + (double)laser_rayCount / 2;
ROS_INFO("2\n");
        int left = obi[0].left_bot_x - image_pixel_width / 2 ;
	//double left_angle = (double)((double)left / (double)image_pixel_width) * (double) camera_horizontal;
	double left_angle = atan2((double)left,(double)220.604700396)*((double)180/PI);
ROS_INFO("3\n");
        //int leftBeam = (double)(((double)left / image_pixel_width) * laser_rayCount * (camera_horizontal/laser_angle)) + (double)laser_rayCount / 2;

        ROS_INFO("angles : %f, %f",left_angle,right_angle);


        int left_beam = (double)((double)((double)laser_rayCount / (double)laser_angle) * left_angle) + (double)(laser_rayCount / 2) ;
ROS_INFO("4\n");
        ROS_INFO("left %d, right %d ",left,right);
        dist = computeDistance(left_beam,right_beam);
        ROS_INFO("distance is : %f",dist);
        ROS_INFO("new victim! \n");
        victims.push_back(getVictimLocation((right_angle + left_angle)/2));
        //Image
	//TODO
        if(false)
          send_message(getVictimLocation((right_angle + left_angle)/2),true);
        else
          send_message(getVictimLocation((right_angle + left_angle)/2),false);
        /** TODO MARKER SHOULD BE HERE */
        return;
    }
    else{
      dist = -1;
      ROS_INFO("dist is set on -1 !\n");
      }
    }
    ROS_INFO("size !> 0\n");
  }
  void robotPoseInitialCallback(const nav_msgs::Odometry::ConstPtr& msg){
    robotX = msg->pose.pose.position.x;
    robotY = msg->pose.pose.position.y;
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    robot_yaw_angle = tf::getYaw(pose.getRotation());
  }
void thermalCallback(const sensor_msgs::Image::ConstPtr& msg){
  imageRows = msg->height;
  imageColumns = msg->step;
  thermalImageArray = msg->data;
  ROS_INFO("heat output : _______________________  %u",thermalImageArray[imageRows * imageColumns / 2]);
}
int main(int argc, char **argv)
{
  std::string thermal_str,laser_str,detector_str,odom_str,dead_victim_detected_str,alive_victim_detected_str;
  ros::init(argc, argv, "position_estimator");
  ros::Time::init();
  ros::Rate pub_rate(20);
  //firstInit();
  ROS_INFO("firstInit called! \n");
  ros::NodeHandle nh;
  ros::Subscriber scanSub;
  ros::Subscriber thermalSub;
  ros::Subscriber victSub;
  ros::Subscriber robotPoseSub;
  // image_pixle_width = 280 : pixels in width
  // laser_rayCount = 720 : number of rays
  // camera_horizontal = 62.8 : horizontal_fov of camera
  // laser_angle = 260 : max_angle - min_angle
  //robot_namespace = "sos1";
  nh.getParam("victim_pose/robot_namespace", robot_namespace);
  nh.getParam("victim_pose/laser_angle", laser_angle);
  nh.getParam("victim_pose/image_pixle_width", image_pixel_width);
  nh.getParam("victim_pose/camera_horizontal", camera_horizontal);
  nh.getParam("victim_pose/laser_rayCount", laser_rayCount);


  laser_str = '/' + robot_namespace + "/remmaped_scan";
  thermal_str = '/' + robot_namespace + "/themral_camera/image";
//  detector_str = '/' + robot_namespace + "/detector_node/detections";
  detector_str = "/detector_node/detections";
  odom_str = '/' + robot_namespace + "/odom";
  dead_victim_detected_str =  '/' + robot_namespace + "/dead_victim_detected";
  alive_victim_detected_str =  '/' + robot_namespace + "/alive_victim_detected";




  thermalSub = nh.subscribe<sensor_msgs::Image>(thermal_str,10,thermalCallback);
  scanSub = nh.subscribe<sensor_msgs::LaserScan>(laser_str,10,processLaserScanCallback);
  victSub = nh.subscribe<aut_exploration::Detections>(detector_str,10,processDetectionsCallback);
  //victSub = nh.subscribe<aut_exploration::Detections>("/detector_node/detections",10,helloWorld);
  robotPoseSub = nh.subscribe<nav_msgs::Odometry>(odom_str,10,robotPoseInitialCallback);
  dead_victPub = nh.advertise<geometry_msgs::Point>(dead_victim_detected_str, 10);
  alive_victPub = nh.advertise<geometry_msgs::Point>(alive_victim_detected_str, 10);

  ROS_INFO("firstInit done! \n");

  int count=0;
  while (ros::ok())
  {

    ros::spinOnce();
    pub_rate.sleep();
  }
  exit(0);
  return 0;
}
