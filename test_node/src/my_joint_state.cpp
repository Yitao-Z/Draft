#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_joint_positions");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  
  gazebo_msgs::SetModelConfiguration srv;
  srv.request.model_name = "robot1";
  srv.request.urdf_param_name = "robot_description";
  srv.request.joint_names = {"Left_Roll", "Left_Pitch", "Left_Slide", "Left_Foot_Pitch", "Left_Foot_Roll", "Right_Roll", "Right_Pitch", "Right_Slide", "Right_Foot_Pitch", "Right_Foot_Roll"};
  srv.request.joint_positions = {0.3, 1.2, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (client.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service set_model_configuration");
    return 1;
  }

  return 0;
}