
// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/topic.h>

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<geometry_msgs::Twist> base_control_client;
typedef boost::shared_ptr< base_control_client>  base_control_client_Ptr;


// Create a ROS action client to move TIAGo's base
void createBaseClient(base_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to base controller ...");

  actionClient.reset( new base_control_client("/mobile_base_controller/cmd_vel") );

  int iterations = 0, max_iterations = 3;
  // Wait for base controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the base_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createBaseClient: base controller action server not available");
}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_base_control");

  ROS_INFO("Starting run_base_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create a base controller action client to move the TIAGo's base
  base_control_client_Ptr BaseClient;
  createBaseClient(BaseClient);

    // Beyblade
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 3.0;
/*
  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  waypoints_arm_goal(arm_goal);


  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }
*/
  return EXIT_SUCCESS;
}