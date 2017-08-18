#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>


namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  physics::WorldPtr _world;
  ros::Publisher polygonPublisher;
  ros::NodeHandle n;
  ros::Subscriber map_subscriber;
  WorldPluginTutorial() : WorldPlugin()
  {

  }
  float get_distance(std::string robot_name1,std:: string robot_name2){
    physics::ModelPtr model1,model2;
     model1=_world->GetModel(robot_name1);
     model2=_world->GetModel(robot_name2);
     if (model1 == 0 || model2 == 0 ){

       ROS_WARN("invalid model name!");
       return -1;
     }

   else {
     float x_diff = model1->GetWorldPose().pos.x - model2->GetWorldPose().pos.x;
     float y_diff = model1->GetWorldPose().pos.y - model2->GetWorldPose().pos.y;

     return sqrt ((x_diff*x_diff)+(y_diff*y_diff));
   }

  }
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {


    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    _world=_parent;

    ROS_INFO("Hello World!");
    ROS_INFO("Hello World!");

    ROS_INFO("Hello World!");



    sdf::SDF sphereSDF;
   sphereSDF.SetFromString(
      "<sdf version ='1.4'>\
         <model name ='sphere'>\
           <pose>-10 -12 0.2 0 0 0</pose>\
           <link name ='link'>\
             <pose>0 0 0 0 0 0</pose>\
             <collision name ='collision'>\
               <geometry>\
                 <sphere><radius>0.05</radius></sphere>\
               </geometry>\
             </collision>\
             <visual name ='visual'>\
               <geometry>\
                 <sphere><radius>0.05</radius></sphere>\
               </geometry>\
             </visual>\
           </link>\
         </model>\
       </sdf>");
   // Demonstrate using a custom model name.
   sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
   model->GetAttribute("name")->SetFromString("unique_sphere");
   _world->InsertModelSDF(sphereSDF);
   ROS_INFO("Hello World!");

   //ros::WallDuration(15.0).sleep();

    //std::string asfj="asphalt_plane";
    //std::string hi=_world->GetModel(asfj)->GetName();
    //ROS_INFO("%s",hi.c_str());
    //ROS_INFO("%f",_world->GetModel(1)->GetWorldPose().pos.x );
    //ROS_INFO("%f",get_distance("asphalt_plane","asphalt_plane_1"));
    //math::Pose asdfjk(20,20,0,0,0,0);
    //_world->GetModel(asfj)->SetWorldPose(asdfjk);

    ROS_INFO("going for node");
    int argc = 0;
 char **argv = NULL;
 ros::init(argc, argv, "gazebo_client",
     ros::init_options::NoSigintHandler);
     ROS_INFO("going for nodehandler");

    ROS_INFO("Hello World!");
    polygonPublisher= n.advertise<std_msgs::String>("publishing_topic",10);
    std::string maptopic="subscribing_topic";//move_base/local_costmap/costmap";
     map_subscriber = n.subscribe(maptopic, 5,mapcallBack);

  }
  void static mapcallBack(const std_msgs::String mapraw){

  ROS_INFO("%s",mapraw.data.c_str());
}

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
