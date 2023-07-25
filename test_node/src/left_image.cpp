#include <ros/ros.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/pcl_config.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <cmath>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::polygon<Point> Polygon;

int main(int argc, char **argv)
{   
    // Link trans
    float theta[5] = {0.3, 1.2, -0.1, 0, 0};

    Eigen::Matrix<float, 16, 4> left_leg;
    left_leg << -0.0205, -0.0205, -0.6, 1, // leg
    0.0205, -0.0205, -0.6, 1,
    0.0205, 0.0205, -0.6, 1,
    -0.0205, 0.0205, -0.6, 1,
    -0.0205, -0.0205, -0.2-theta[2], 1,
    0.0205, -0.0205, -0.2-theta[2], 1,
    0.0205, 0.0205, -0.2-theta[2], 1,
    -0.0205, 0.0205, -0.2-theta[2], 1,
    -0.105, -0.035, -0.43, 1, // wheel
    0.105, -0.035, -0.43, 1,
    0.105, 0.038, -0.43, 1,
    -0.105, 0.038, -0.43, 1,
    -0.105, -0.035, -0.35, 1,
    0.105, -0.035, -0.35, 1,
    0.105, 0.038, -0.35, 1,
    -0.105, 0.038, -0.35, 1;

    Eigen::Matrix4f left_roll;
    left_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.075f,
                0.0f, 0.0f, 1.0f, 0.591116f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c0 = std::cos(theta[0]);
    float s0 = std::sin(theta[0]);
    Eigen::Matrix4f T_left_roll;
    T_left_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, c0, s0, 0.0,
                0.0f, -s0, c0, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f left_pitch;
    left_pitch << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c1 = std::cos(-theta[1]);
    float s1 = std::sin(-theta[1]);
    Eigen::Matrix4f T_left_pitch;
    T_left_pitch << c1, 0.0f, s1, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                -s1, 0.0f, c1, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f left_slide;
    left_slide << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.1042f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;


    Eigen::Matrix4f T_left_slide;
    T_left_slide << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, theta[2],
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f trans;
    trans = left_roll * T_left_roll * left_pitch * T_left_pitch * left_slide * T_left_slide;

    Eigen::Matrix<float, 16, 4> output =  left_leg * trans.transpose();

    //std::cout << "output:\n" << output << std::endl;
    
    Eigen::Matrix<float, 3, 4>  extrinsic;
    extrinsic << 0.7071f, 0.0f, -0.7071f, 0.3748f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.7071, 0, 0.7071, -0.5445;

    Eigen::Matrix<float, 3, 3>  intrinsic;
    intrinsic << 381.3624f, 0.0f, 320.5f,
                0.0f, 381.3624f, 240.5f,
                0.0f, 0.0f, 1.0f;
    
    Eigen::Matrix<float, 3, 3>  R;
    R << 0.0f, 0.0f, 1.0f,
                -1.0f, 0.0f, 0.0f,
                0.0f, -1.0f, 0.0f;

    Eigen::Matrix<float, 16, 3> camera_coords = output * extrinsic.transpose() * R;
    Eigen::Matrix<float, 16, 3> raw_pixels = camera_coords * intrinsic.transpose();
    for (int i = 0; i < raw_pixels.rows(); ++i) {
        if (raw_pixels(i, raw_pixels.cols() - 1) < 0){
            raw_pixels(i, raw_pixels.cols() - 1) = 0.01;
        }
    }
    raw_pixels.col(0) = raw_pixels.col(0).array() / raw_pixels.col(2).array();
    raw_pixels.col(1) = raw_pixels.col(1).array() / raw_pixels.col(2).array();
    Eigen::Matrix<int, 16, 3> int_pixels = raw_pixels.array().floor().matrix().cast<int>();
    //std::cout << "output:\n" << int_pixels << std::endl;

    std::vector<Point> clipping_polygon = {
        {0, 0},    // Clipping polygon vertex 0
        {640, 0},    // Clipping polygon vertex 1
        {640, 480},    // Clipping polygon vertex 2
        {0, 480},    // Clipping polygon vertex 3
        {0, 0}     // Closing point
    };

    Eigen::Matrix<int, 7, 4> surf_vertex_index;
    surf_vertex_index << 0, 1, 5, 4,
                1, 2, 6, 5,
                0, 3, 7, 4,
                8, 9, 13, 12,
                9, 10, 14, 13,
                8, 11, 15, 12,
                12, 13, 14, 15;

    cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);
    
    std::cout << int_pixels << std::endl;

    for(int i = 0; i < surf_vertex_index.rows(); ++i)
    {   
        std::cout << surf_vertex_index.row(i) << std::endl;
        std::cout << int_pixels(surf_vertex_index(i,0),0) <<std::endl;
        std::cout << int_pixels(surf_vertex_index(i,1),0) <<std::endl;
        std::cout << int_pixels(surf_vertex_index(i,2),0) <<std::endl;
        

        if (int_pixels(surf_vertex_index(i,0),1)==int_pixels(surf_vertex_index(i,2),1)){
            continue;
        }

        if ((int_pixels(surf_vertex_index(i,0),0)<0 || int_pixels(surf_vertex_index(i,0),1)<0) &&
            (int_pixels(surf_vertex_index(i,1),0)<0 || int_pixels(surf_vertex_index(i,1),1)<0) &&
            (int_pixels(surf_vertex_index(i,2),0)<0 || int_pixels(surf_vertex_index(i,2),1)<0) &&
            (int_pixels(surf_vertex_index(i,3),0)<0 || int_pixels(surf_vertex_index(i,3),1)<0))
        {   
            continue;
        }

        

        std::vector<Point> subject_polygon = {
            {int_pixels(surf_vertex_index(i,0),0), int_pixels(surf_vertex_index(i,0),1)},    // Subject polygon vertex 0
            {int_pixels(surf_vertex_index(i,1),0), int_pixels(surf_vertex_index(i,1),1)},    // Subject polygon vertex 1
            {int_pixels(surf_vertex_index(i,2),0), int_pixels(surf_vertex_index(i,2),1)},    // Subject polygon vertex 2
            {int_pixels(surf_vertex_index(i,3),0), int_pixels(surf_vertex_index(i,3),1)},
            {int_pixels(surf_vertex_index(i,0),0), int_pixels(surf_vertex_index(i,0),1)}     // Closing point
        };

        Polygon clipping_poly, subject_poly;
        bg::assign_points(clipping_poly, clipping_polygon);
        bg::correct(clipping_poly); // Fix the orientation of the clipping polygon
        bg::assign_points(subject_poly, subject_polygon);
        bg::correct(subject_poly); // Fix the orientation of the subject polygon

        // Perform Sutherland-Hodgman polygon clipping
        std::vector<Polygon> output;
        bg::intersection(clipping_poly, subject_poly, output);

        //std::cout << output << std::endl;

        size_t total_points = 0;
        for(const auto& polygon : output) {
            total_points += polygon.outer().size();
        }

        Eigen::MatrixXd Points(total_points, 2);
        size_t row = 0;
        for(const auto& polygon : output) {
            for(const auto& point : polygon.outer()) {
                Points(row, 0) = point.x();
                Points(row, 1) = point.y();
                ++row;
            }
        }

        std::vector<cv::Point> pts;
        for(int i = 0; i < Points.rows(); ++i)
        {
            pts.push_back(cv::Point(Points(i, 0), Points(i, 1)));
        }

        std::cout << pts << std::endl;

        std::vector<std::vector<cv::Point>> pts_all;
        pts_all.push_back(pts);
        cv::fillPoly(mask, pts_all, cv::Scalar(255));

    }

    //cv::imshow("Image", mask);
    //cv::waitKey(0);
    ros::init(argc, argv, "left_image");
    ros::NodeHandle nh;
    ros::Publisher pub2 = nh.advertise<sensor_msgs::Image>("mask",10);
    
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_topic", 1);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    
    while (true) {
        pub.publish(msg);
        pub2.publish(msg);
    }
    

    

    return 0;
}
