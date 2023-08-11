#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <pcl/filters/frustum_culling.h>


cv::Mat global_image;
Eigen::Vector3f up;
ros::Publisher pub;
ros::Publisher pub_test;
pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud;
image_transport::Publisher pub_it;

Eigen::Matrix4f trans;
Eigen::Matrix4f m_p;
Eigen::Matrix4f m_r;

void fastNormalEstimation (const cv::Mat& depth_image, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{   
    // global_image = cv::Mat::zeros(240, 424, CV_8UC1);
    global_image = cv::Mat::zeros(480, 640, CV_8UC1);
    test_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    float fx, fy, cx, cy;
    // fx = 216.332153;//381.3624; // Focal length in x
    // fy = 216.332153;// Focal length in y
    // cx = 213.006653; //320.5; // Principal point x
    // cy = 116.551689;//240.5; // Principal point y
    fx = 381.3624; // Focal length in x
    fy = 381.3624;// Focal length in y
    cx = 320.5; // Principal point x
    cy = 240.5; // Principal point y
    float camera_factor = 1;  // for the openni camera !

    float max = 0;

    for (int m = 1; m < depth_image.rows-1; m++)
    {
        for (int n = 1; n < depth_image.cols-1; n++)
        {
            // Get the value at (m, n) in the depth map
            float d1 = depth_image.ptr<float>(m)[n];
            float d2 = depth_image.ptr<float>(m-1)[n]; // down
            float d3 = depth_image.ptr<float>(m)[n+1]; // right
            float d4 = depth_image.ptr<float>(m+1)[n]; // up
            float d5 = depth_image.ptr<float>(m)[n-1]; // left
            // d may have no value, if so, skip this point
            if (std::isnan(d1)||std::isnan(d2)||std::isnan(d3)||std::isnan(d4)||std::isnan(d5))
                continue;

            // point cloud
            pcl::PointXYZ p;
            p.z = double(d1) / camera_factor;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            cloud->points.push_back(p);

            pcl::Normal p_n;
            float nx = (-d3 + d5) * (d2 + d4) / (fy);
            float ny = (-d2 + d4) * (d3 + d5) / (fx);
            float nz = (n - cx)*nx / fx - (m - cy)*ny/ fy - (d2 + d4) * (d3 + d5) / (fx*fy);
            Eigen::Vector3f v;
            v << nx,
                ny,
                nz;
            Eigen::Vector3f v_normalized = v.normalized();
            p_n.normal_x = v_normalized(0);
            p_n.normal_y = v_normalized(1);
            p_n.normal_z = v_normalized(2);

            normals->points.push_back(p_n);
            float temp = v.norm();
            if (temp>max){
                max = temp;
            }
            //std::cout << "max " << max << std::endl;

            // Eigen::Vector3f a24;
            // a24 << (n - cx) * (d2 - d4) / fx,
            //     ((m-1 - cy)*d2 - (m+1 - cy)*d4) / fy,
            //     d2-d4;

            // Eigen::Vector3f a35;
            // a35 << ((n+1 - cx)*d3 - (n-1 - cx)*d5) / fx,
            //     (m - cy) * (d3 - d5) / fy,
            //     d3-d5;  

            // Eigen::Vector3f cross_product = a24.cross(a35);
            // std::cout << "cross_product " << cross_product << std::endl;
            // cross_product.normalize();

            if (temp>0.001){
                continue;
            }

            float dot_product = std::fabs(up.dot(v_normalized)); //v_normalized
            //std::cout << "v " << v << std::endl;
            if (dot_product>0.9){
                global_image.at<uchar>(m, n) = 255;
                test_cloud->push_back(p);
            }

            
        }
    }
    cv::Mat labels;
    int nComponents = cv::connectedComponents(global_image, labels);
    labels = std::floor(255/nComponents)* labels;
    //std::cout << "v " << nComponents << std::endl;
    //std::cout << "v " << global_image.size() << std::endl;
    labels.convertTo(labels, CV_8U);
    cv::Mat coloredLabels;
    cv::applyColorMap(labels, coloredLabels, cv::COLORMAP_JET);

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", global_image).toImageMsg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", labels).toImageMsg();
    pub_it.publish(msg);

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

    // // Convert the cv::Mat to sensor_msgs::PointCloud2
    // sensor_msgs::PointCloud2 cloud = MatToPointCloud(cv_ptr->image);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    fastNormalEstimation(cv_ptr->image, cloud, normals);

    pcl::transformPointCloud(*cloud, *cloud, (m_p*m_r*trans).inverse());
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    //cloud_msg.header.frame_id = "camera_link_fake";
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();

    // Then publish the point cloud.
    pub.publish(cloud_msg);

    pcl::transformPointCloud(*test_cloud, *test_cloud, (m_p*m_r*trans).inverse());
    sensor_msgs::PointCloud2 test_cloud_msg;
    pcl::toROSMsg(*test_cloud, test_cloud_msg);
    //test_cloud_msg.header.frame_id = "camera_link_fake";
    test_cloud_msg.header.frame_id = "world";
    test_cloud_msg.header.stamp = ros::Time::now();

    pub_test.publish(test_cloud_msg);
}


int main(int argc, char **argv)
{   
    // rviz frame to pcl frame
    Eigen::Matrix4f trans_1;
    trans_1 << 0.0f, 0.0f, 1.0f, 0.0f,
              1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, -1.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 0.0f, 1.0f;

    float angle = 0.7854;
    float s_theta = std::sin(angle);
    float c_theta = std::cos(angle);
    
    Eigen::Matrix4f trans_2;
    trans_2 << c_theta, 0.0f, s_theta, 0.0f,
              0.0f, 1.0f, 0.0f, 0.0f,
              -s_theta, 0.0f, c_theta, 0.0f,
              0.0f, 0.0f, 0.0f, 1.0f;
    trans = (trans_2*trans_1).inverse();
    Eigen::Vector4f up4 (0,0,1,1);
    up = (trans * up4).head<3>();

    // Eigen::Vector3f acc (0.114, -8.530, -4.943);
    // Eigen::Vector3f acc_n = acc.normalized();
    // float roll = -3.1416+acos(acc_n(1));
    // float pitch = acos(acc_n(2)/sin(-roll));
    
    // m_p << cos(pitch), 0.0f, sin(pitch), 0.0f,
    //         0.0f, 1.0f, 0.0f, 0.0f,
    //         -sin(pitch), 0.0f, cos(pitch), 0.0f,
    //         0.0f, 0.0f, 0.0f, 1.0f;

    // m_r << 1.0f, 0.0f, 0.0f, 0.0f,
    //     0.0f, cos(roll), -sin(roll), 0.0f,
    //     0.0f, sin(roll), cos(roll), 0.0f,
    //     0.0f, 0.0f, 0.0f, 1.0f;
    // trans = (trans_1).inverse();

    // Eigen::Vector4f up4 (0,0,1,1);
    // up = (m_p*m_r*trans * up4).head<3>();

    
    // std::cout << "up " << trans * up4 << std::endl;
    // std::cout << pitch << std::endl;
    // std::cout << roll << std::endl;

    std::cout << up << std::endl;

    




    ros::init(argc, argv, "fast_normal");

    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("fast_normal",10);
    pub_test = nh.advertise<sensor_msgs::PointCloud2>("test_cloud",10);
    
    

    image_transport::ImageTransport it(nh);
    pub_it = it.advertise("fast_normal_image", 1);
    
    // /camera/depth/image_raw
    // /depth_publisher
    ros::Subscriber sub = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallback);
    

    ros::spin();

    return 0;
}