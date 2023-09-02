#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <pcl/filters/frustum_culling.h>
#include <vector>


cv::Mat global_image;
Eigen::Vector3f up;
ros::Publisher pub;
ros::Publisher pub_test;
pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud;
image_transport::Publisher pub_it;

Eigen::Matrix4f trans_1;
Eigen::Matrix4f trans_2;
Eigen::Matrix4f trans_2p;
Eigen::Matrix4f trans;
Eigen::Matrix4f m_p;
Eigen::Matrix4f m_r;


void fastNormalEstimation (const cv::Mat& depth_image, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{   
    global_image = cv::Mat::zeros(240, 424, CV_8UC1);
    //global_image = cv::Mat::zeros(480, 640, CV_8UC1);
    test_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    float fx, fy, cx, cy;
    fx = 216.332153;//381.3624; // Focal length in x
    fy = 216.332153;// Focal length in y
    cx = 213.006653; //320.5; // Principal point x
    cy = 116.551689;//240.5; // Principal point y
    // fx = 381.3624; // Focal length in x
    // fy = 381.3624;// Focal length in y
    // cx = 320.5; // Principal point x
    // cy = 240.5; // Principal point y
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
    cv::Mat stats;
    cv::Mat centroids;
    //int nComponents = cv::connectedComponents(global_image, labels);
    int nComponents = cv::connectedComponentsWithStats(global_image, labels, stats, centroids);
    std::cout << "nComponents " << nComponents << std::endl;
    std::cout << "stats " << stats << std::endl;
    std::cout << "centroids " << centroids << std::endl;
    //std::cout << "centroids " << static_cast<int>(std::round(centroids.at<double>(2,1))) << std::endl;
    // Merge
    float z_values[nComponents];
    for (int i=1; i<nComponents; i++){
        double centroid_x = centroids.at<double>(i,0);
        double centroid_y = centroids.at<double>(i,1);
        float d = depth_image.ptr<float>(static_cast<int>(centroid_y))[static_cast<int>(centroid_x)];
        float z_c = double(d);
        float x_c = (centroid_x - 213.006653) * z_c / 216.332153;
        float y_c = (centroid_y - 116.551689) * z_c / 216.332153;
        Eigen::Vector4f v_c (x_c, y_c, z_c, 1);
        float z_val = (trans.inverse()*v_c)(2);
        z_values[i] = z_val;
    }
    std::cout << "z_values " << std::endl;
    for (const auto& item : z_values) {
        std::cout << item << " ";
    }

    std::vector<std::vector<int>> new_z_list = {};

    for (int i=1; i<nComponents; i++){
        bool matched = false;
        for (int j = 0; j < new_z_list.size(); j++) {
            if (std::fabs(z_values[new_z_list[j][0]] - z_values[i]) < 0.02) {
                new_z_list[j].push_back(i);
                matched = true;
                break;
            }
        }

        if (!matched) {
            new_z_list.push_back({i});
        }

    }

    std::set<int> removedIndices;
    new_z_list.erase(
        std::remove_if(
            new_z_list.begin(), 
            new_z_list.end(), 
            [&stats, &removedIndices](const std::vector<int>& inner) { 
                int sum = 0;
                for (int index : inner) {
                    sum += stats.at<int>(index,4);
                }
                if (sum < 100){
                    removedIndices.insert(inner.begin(), inner.end());
                }
                return sum < 100;
            }),
        new_z_list.end()
    );

    std::vector<float> new_z_values = {};

    for (const auto& innerList : new_z_list) {
        new_z_values.push_back(z_values[innerList[0]]);
        if (innerList.size() > 1) {
            int firstValue = innerList[0];

            // Replace all elements in innerList with firstValue in labels
            for (int val : innerList) {
                labels.setTo(firstValue, labels == val);
            }
        }

    }

    for (int value : removedIndices) {
        labels.setTo(0, labels == value);
    }


    // sort
    std::vector<size_t> indices(new_z_values.size());
    for (size_t i = 0; i < new_z_values.size(); ++i) {
        indices[i] = i;
    }

    std::sort(indices.begin(), indices.end(), 
              [&new_z_values](size_t i1, size_t i2) {
                  return new_z_values[i1] < new_z_values[i2];
              });

    std::vector<std::vector<int>> sorted_new_z_list(new_z_list.size());
    std::vector<float> sorted_new_z_values(new_z_values.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        sorted_new_z_list[i] = new_z_list[indices[i]];
        sorted_new_z_values[i] = new_z_values[indices[i]];
    }

    new_z_list = sorted_new_z_list;
    new_z_values = sorted_new_z_values;

    std::cout << "new_z_list " << std::endl;
    for (const auto& inner_vec : new_z_list) {
        for (int num : inner_vec) {
            std::cout << num << " ";
        }
        std::cout << std::endl;  // New line for each inner vector
    }

    std::cout << "new_z_values " << std::endl;
    for (const auto& item : new_z_values) {
        std::cout << item << " ";
    }

    

    labels = std::floor(255/nComponents)* labels;


    labels.convertTo(labels, CV_8U);
    cv::Mat coloredLabels;
    cv::applyColorMap(labels, coloredLabels, cv::COLORMAP_JET);

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", global_image).toImageMsg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", labels).toImageMsg();
    pub_it.publish(msg);

}





void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

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

    //pcl::transformPointCloud(*cloud, *cloud, (m_p*m_r*trans).inverse());
    pcl::transformPointCloud(*cloud, *cloud, (m_p*m_r*trans).inverse());
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    //cloud_msg.header.frame_id = "camera_link_fake";
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();

    // Then publish the point cloud.
    pub.publish(cloud_msg);

    //pcl::transformPointCloud(*test_cloud, *test_cloud, trans_2p*trans.inverse());
    //pcl::transformPointCloud(*test_cloud, *test_cloud, (m_p*m_r*trans).inverse());
    pcl::transformPointCloud(*test_cloud, *test_cloud, trans.inverse());
    sensor_msgs::PointCloud2 test_cloud_msg;
    pcl::toROSMsg(*test_cloud, test_cloud_msg);
    //test_cloud_msg.header.frame_id = "camera_link_fake";
    test_cloud_msg.header.frame_id = "base_link";
    test_cloud_msg.header.stamp = ros::Time::now();

    pub_test.publish(test_cloud_msg);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> t4 = 1000*(end - begin);
    std::cout << "time: " << t4.count() << " ms" << std::endl;
}


int main(int argc, char **argv)
{   
    // rviz frame to pcl frame
    trans_1 << 0.0f, 0.0f, 1.0f, 0.0f,
              -1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, -1.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 0.0f, 1.0f;

    float angle = 0.7854;
    float s_theta = std::sin(angle);
    float c_theta = std::cos(angle);

    // trans_2p << 1.0f, 0.0f, 0.0f, 0.12f,
    //         0.0f, 1.0f, 0.0f, 0.0f,
    //         0.0f, 0.0f, 1.0f, 0.1f,
    //         0.0f, 0.0f, 0.0f, 1.0f;
    
    // trans_2 << c_theta, 0.0f, s_theta, 0.0f,
    //           0.0f, 1.0f, 0.0f, 0.0f,
    //           -s_theta, 0.0f, c_theta, 0.0f,
    //           0.0f, 0.0f, 0.0f, 1.0f;
    // trans = (trans_2*trans_1).inverse();
    // //trans = trans_2.inverse()*trans_1;
    // Eigen::Vector4f up4 (0,0,1,1);
    // up = (trans * up4).head<3>();

    Eigen::Vector4f acc (0.114, -8.530, -4.943, 1);
    Eigen::Vector3f acc_n = acc.head<3>().normalized();
    // float roll = -3.1416+acos(acc_n(1));
    // float pitch = acos(acc_n(2)/sin(-roll));
    // trans = (trans_1).inverse();

    Eigen::Vector4f new_acc = trans_1*acc;
    float roll = atan(-new_acc(1)/sqrt(new_acc(0)*new_acc(0)+new_acc(2)*new_acc(2)));
    float pitch = atan(new_acc(0)/sqrt(new_acc(1)*new_acc(1)+new_acc(2)*new_acc(2)));
    m_r << 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, cos(roll), -sin(roll), 0.0f,
        0.0f, sin(roll), cos(roll), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    m_p << cos(pitch), 0.0f, sin(pitch), 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            -sin(pitch), 0.0f, cos(pitch), 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
    trans = (trans_1.inverse()*m_p*m_r);
    //trans = (trans_1).inverse();

    Eigen::Vector4f up4 (0,0,1,1);
    //up = (m_p*m_r*trans * up4).head<3>();
    up = (trans* up4).head<3>();
    
    std::cout << "up " << trans * up4 << std::endl;
    std::cout << pitch << std::endl;
    std::cout << roll << std::endl;

    std::cout << up << std::endl;



    ros::init(argc, argv, "fast_normal");

    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("fast_normal",10);
    pub_test = nh.advertise<sensor_msgs::PointCloud2>("test_cloud",10);
    
    

    image_transport::ImageTransport it(nh);
    pub_it = it.advertise("fast_normal_image", 1);
    
    // /camera/depth/image_raw
    // /depth_publisher
    ros::Subscriber sub = nh.subscribe("/depth_publisher", 1, depthImageCallback);
    

    ros::spin();

    return 0;
}