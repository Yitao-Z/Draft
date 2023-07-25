#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>

cv::Mat global_image;
ros::Publisher pub;

void maskCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        global_image = cv_bridge::toCvShare(msg, "mono8")->image;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        global_image = cv_ptr->image;
        //std::cout << "global_image 1 " << global_image.ptr<uchar>(450)[450] << std::endl;
        //std::cout << "global_image 2 " << global_image.ptr<uchar>(0)[0] << std::endl;
        //std::cout << "global_image 3 " << global_image.ptr<float>(100)[0] << std::endl;


        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '???'.", msg->encoding.c_str());
    }
}

sensor_msgs::PointCloud2 MatToPointCloud(const cv::Mat& depth_image)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    // Assuming you know the camera parameters (focal length, principal point (cx, cy), camera factor)
    float fx, fy, cx, cy;
    fx = 381.3624; // Focal length in x
    fy = 381.3624; // Focal length in y
    cx = 320.5; // Principal point x
    cy = 240.5; // Principal point y
    float camera_factor = 1;  // for the openni camera !
    //std::cout << "depth_image.cols " << depth_image.cols << std::endl;
    //std::cout << "d " << depth_image.ptr<float>(240)[320] << std::endl;
    //std::cout << "d " << depth_image.ptr<ushort>(479)[639] << std::endl;
    
    for (int m = 0; m < depth_image.rows; m++)
    {
        for (int n = 0; n < depth_image.cols; n++)
        {
            // Get the value at (m, n) in the depth map
            float d = depth_image.ptr<float>(m)[n];
            // d may have no value, if so, skip this point
            if (std::isnan(d))
                continue;

            uchar mask_pixel_value = global_image.at<uchar>(m,n);
            //std::cout << static_cast<int>(mask_pixel_value) << std::endl;
            if (mask_pixel_value == 255)
                continue;

            // If d has a value, then create a point cloud
            pcl::PointXYZ p;

            // Calculate the space coordinates of this point
            p.z = double(d) / camera_factor;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            // Add p to the point cloud
            cloud->points.push_back(p);
        }
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;  // since it is not an organized point cloud

    //std::cout << "cloud->width " << cloud->width << std::endl;

    // Convert the pcl::PointCloud to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);

    // Set the frame ID to the frame of your choice
    output.header.frame_id = "camera_link_fake";
    output.header.stamp = ros::Time::now();

    return output;
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the sensor_msgs/Image to cv::Mat.
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); //TYPE_16UC1
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert the cv::Mat to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud = MatToPointCloud(cv_ptr->image);

    // Then publish the point cloud.
    pub.publish(cloud);
}



int main(int argc, char **argv)
{   
    ros::init(argc, argv, "depth2pc");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_mask = it.subscribe("/image_topic", 1, maskCallback);

    ros::Subscriber sub = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallback);

    
    pub = nh.advertise<sensor_msgs::PointCloud2>("depth2pc",10);

    ros::spin();

    return 0;
}