#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <fstream>
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_publisher");
    ros::NodeHandle nh;

    std::string filename = "/home/yitao/ws/src/test_mask/src/1depth_Depth.raw";
    int width = 640;
    int height = 480;

    // Open the file
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        ROS_ERROR("Could not open the file!");
        return 1;
    }

    // Read the data
    std::vector<uint16_t> depth_image_mm(width * height);
    file.read(reinterpret_cast<char*>(depth_image_mm.data()), depth_image_mm.size() * sizeof(uint16_t));

    // Check for reading errors
    if (!file) {
        ROS_ERROR("An error occurred while reading the file!");
        return 1;
    }

    // Create an Image message
    sensor_msgs::Image depth_image_msg;
    depth_image_msg.header.frame_id = "depth_frame";
    depth_image_msg.height = height;
    depth_image_msg.width = width;
    depth_image_msg.encoding = "32FC1";
    depth_image_msg.is_bigendian = false;
    depth_image_msg.step = width * sizeof(float);
    depth_image_msg.data.resize(width * height * sizeof(float));

    // Convert to floating-point and scale to meters
    float* data_ptr = reinterpret_cast<float*>(depth_image_msg.data.data());
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            data_ptr[i * width + j] = depth_image_mm[i * width + j] / 1000.0f;
        }
    }

    // Publish the image on a topic
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("depth_publisher", 1);
    ros::Rate loop_rate(30); // Adjust the rate as needed
    while (ros::ok()) {
        depth_image_msg.header.stamp = ros::Time::now();
        pub.publish(depth_image_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
