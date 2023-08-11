#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <Eigen/Dense>

void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{   
    // "slider_gazebo::base_link"
    std::string link_name = "slider_gazebo::base_link"; // the name of the link
    auto it = std::find(msg->name.begin(), msg->name.end(), link_name);
    // ROS_INFO("Found link: %s", it->c_str());
    // for (const auto& name : msg->name)
    // {
    //     ROS_INFO("%s", name.c_str());
    // }

    if (it != msg->name.end())
    {
        int index = std::distance(msg->name.begin(), it);
        std::cout << "Here is the matrix index:\n" << index << std::endl;
        const geometry_msgs::Quaternion& orientation = msg->pose[index].orientation;

        tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 rotation_matrix(quaternion);
        std::cout << "Rotation Matrix:\n"
          << rotation_matrix[0][0] << " " << rotation_matrix[0][1] << " " << rotation_matrix[0][2] << "\n"
          << rotation_matrix[1][0] << " " << rotation_matrix[1][1] << " " << rotation_matrix[1][2] << "\n"
          << rotation_matrix[2][0] << " " << rotation_matrix[2][1] << " " << rotation_matrix[2][2] << std::endl;

        Eigen::Matrix4f trans;
        trans << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], 0.0f,
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], 0.0f,
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;

        std::cout << "Here is the matrix trans:\n" << trans << std::endl;

        
    }
    else
    {
        ROS_WARN("Link not found!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "link_orientation_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 1000, linkStatesCallback);

    ros::spin();

    return 0;
}
