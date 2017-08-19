#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "sosvr_gazebo_plugins/distance_serivce.h"
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
    ros::ServiceServer service ;
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
//boost::bind(&MoveBase::goalCB, this, _1)
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
        service = n.advertiseService("distance_serivce",  &WorldPluginTutorial::service_handler,this);
        polygonPublisher= n.advertise<std_msgs::String>("publishing_topic",10);
        std::string maptopic="subscribing_topic";//move_base/local_costmap/costmap";
        map_subscriber = n.subscribe(maptopic, 5,&WorldPluginTutorial::mapcallBack,this);

      }
      void  mapcallBack(const std_msgs::String mapraw){

        ROS_INFO("%s",mapraw.data.c_str());
      }

      bool  service_handler(sosvr_gazebo_plugins::distance_serivce::Request  &req,
         sosvr_gazebo_plugins::distance_serivce::Response &res)
         {
          std::string asfj="unique_sphere";
          math::Pose asdfjk(20,20,0,0,0,0);
          _world->GetModel(asfj)->SetWorldPose(asdfjk);

         std::string command =req.command,robot1 =req.robot1,robot2 =req.robot2;
         if (command == "distance"){
           res.distance= get_distance(robot1,robot2);
           res.number_of_objects=0;
         }

      return true;
      }


    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
  }
