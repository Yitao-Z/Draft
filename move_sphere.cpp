#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_sphere");
  ros::NodeHandle nh;

  // Service client for setting model state
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::Rate loop_rate(30);

  double initial_t = 0;
  int flag = 1;

  while (ros::ok())
  { 
    ros::Time sim_time = ros::Time::now(); // Get the current simulation time

    if (!ros::Time::now().isZero() && flag==1)
    {
      initial_t = sim_time.toSec();
      flag = 0;
    }

    // Calculate the new position as a function of time
    double t = sim_time.toSec() - initial_t;
    double x = 0.3*std::sin(t)+0.3*t+1;
    std::cout << "t " << t << std::endl;
    std::cout << "x " << x << std::endl;
    double y = 0.0;
    double z = 0.1;

    // Define the new pose for the sphere
    geometry_msgs::Pose new_pose;
    new_pose.position.x = x;
    new_pose.position.y = y;
    new_pose.position.z = z;

    // Create the SetModelState service message
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = "my_sphere"; // Name of the model
    srv.request.model_state.pose = new_pose;

    // Call the service to update the model's state
    if (client.call(srv))
    {
      ROS_INFO("Successfully moved sphere!");
    }
    else
    {
      ROS_ERROR("Failed to call service set_model_state");
    }

    loop_rate.sleep(); // Wait to maintain the loop rate
  }

  return 0;
}
