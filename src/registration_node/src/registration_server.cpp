#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <registration_node/RegistrationAction.h>

class RegistrationAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<registration_node::RegistrationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  registration_node::RegistrationFeedback feedback_;
  registration_node::RegistrationResult result_;
  // Construct a string vector of name of robots registered for parameter server
  std::vector<std::string> robots_list_;
  // counter of robots registered
  int robot_counter_;

public:

  RegistrationAction(std::string name) :
    as_(nh_, name, boost::bind(&RegistrationAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    robot_counter_ = 0;
    robots_list_.clear();
  }

  ~RegistrationAction(void)
  {
  }

  void executeCB(const registration_node::RegistrationGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // checking some conditions ...
    // TODO
    
    // status sequence
    std::string temp = "registering " ;
    feedback_.status = temp + goal->robot_namespace.c_str();;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, registering robot: %s", action_name_.c_str(), goal->robot_namespace.c_str());

    // start executing the action
    // add robto_namespace to the robots_list and increase robot_counter by one
    robots_list_.push_back(goal->robot_namespace);
    
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      // break;
    }
    
    // add registered robot to the parameter robots_list
    nh_.setParam("robots_list", robots_list_);
    // publish the feedback
    as_.publishFeedback(feedback_);
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();

    if(success)
    {
      result_.robots_list = robots_list_;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "registration");
  ROS_INFO("registration server started");
  RegistrationAction registration("registration");
  ros::spin();

  return 0;
}