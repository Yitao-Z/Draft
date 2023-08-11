#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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

#include <chrono>


namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::polygon<Point> Polygon;

cv::Mat mask;

void draw_mask(const std::vector<Point>& clipping_polygon, const Eigen::MatrixXi& int_pixels, 
const Eigen::MatrixXi& surf_vertex_index)
{   
    for(int i = 0; i < surf_vertex_index.rows(); ++i)
    {   
        if ((int_pixels(surf_vertex_index(i,0),1)==int_pixels(surf_vertex_index(i,2),1)) ||
            (int_pixels(surf_vertex_index(i,0),0)==int_pixels(surf_vertex_index(i,2),0)))
        {
            continue;
        }

        // if (((int_pixels(surf_vertex_index(i,0),0)<0 || int_pixels(surf_vertex_index(i,0),0)>639) ||
        //     (int_pixels(surf_vertex_index(i,0),1)<0 || int_pixels(surf_vertex_index(i,0),1)>639)) &&
        //     ((int_pixels(surf_vertex_index(i,1),0)<0 || int_pixels(surf_vertex_index(i,1),0)>639) ||
        //     (int_pixels(surf_vertex_index(i,1),1)<0 || int_pixels(surf_vertex_index(i,1),1)>639)) &&
        //     ((int_pixels(surf_vertex_index(i,2),0)<0 || int_pixels(surf_vertex_index(i,2),0)>639) ||
        //     (int_pixels(surf_vertex_index(i,2),1)<0 || int_pixels(surf_vertex_index(i,2),1)>639)) &&
        //     ((int_pixels(surf_vertex_index(i,3),0)<0 || int_pixels(surf_vertex_index(i,3),0)>639) ||
        //     (int_pixels(surf_vertex_index(i,3),1)<0 || int_pixels(surf_vertex_index(i,3),1)>639)))
        // {   
        //     continue;
        // }

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

        if (!bg::intersects(clipping_poly, subject_poly)){
            continue;
        }

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

        //std::cout << pts << std::endl;

        std::vector<std::vector<cv::Point>> pts_all;
        pts_all.push_back(pts);
        cv::fillPoly(mask, pts_all, cv::Scalar(255));
    }
}

int main(int argc, char **argv)
{   
    // input: Link trans
    float theta[10] = {0.3, 1.2, 0.26, -0.5, -0.5, -0., -0.0, -0., 0., 0.};
    //auto start = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::steady_clock::now();
    // vertex of leg and foot
    Eigen::Matrix<float, 16, 4> leg;
    leg << -0.025, -0.021, 0, 1, // leg
    0.021, -0.021, 0, 1,
    0.021, 0.021, 0, 1,
    -0.025, 0.021, 0, 1,
    -0.025, -0.021, 0.4, 1,
    0.021, -0.021, 0.4, 1,
    0.021, 0.021, 0.4, 1,
    -0.025, 0.021, 0.4, 1,
    -0.119, -0.054, 0.128, 1, // wheel
    0.041, -0.054, 0.128, 1,
    0.041, 0.054, 0.128, 1,
    -0.119, 0.054, 0.128, 1,
    -0.119, -0.054, 0.243, 1,
    0.041, -0.054, 0.243, 1,
    0.041, 0.054, 0.243, 1,
    -0.119, 0.054, 0.243, 1;

    Eigen::Matrix<float, 8, 4> foot;
    foot << 0.115, -0.085, -0.05, 1,
    -0.1155, -0.085, -0.05, 1,
    -0.1155, -0.085, 0.0015, 1,
    0.115, -0.085, 0.0015, 1,
    0.115, 0.0855, -0.05, 1,
    -0.1155, 0.0855, -0.05, 1,
    -0.1155, 0.0855, 0.0015, 1,
    0.115, 0.0855, 0.0015, 1;

    // joint trans
    // left leg
    Eigen::Matrix4f left_roll;
    left_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.075f,
                0.0f, 0.0f, 1.0f, 0.0f,
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
                0.0f, 1.0f, 0.0f, 0.1042f,
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
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, -0.7f,
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f T_left_slide;
    T_left_slide << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, theta[2],
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f left_foot_pitch;
    left_foot_pitch << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c3 = std::cos(-theta[3]);
    float s3 = std::sin(-theta[3]);
    Eigen::Matrix4f T_left_foot_pitch;
    T_left_foot_pitch << c3, 0.0f, s3, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                -s3, 0.0f, c3, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
        
    Eigen::Matrix4f left_foot_roll;
    left_foot_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c4 = std::cos(theta[4]);
    float s4 = std::sin(theta[4]);
    Eigen::Matrix4f T_left_foot_roll;
    T_left_foot_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, c0, s0, 0.0,
                0.0f, -s0, c0, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    // right leg
    Eigen::Matrix4f right_roll;
    right_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, -0.075f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c5 = std::cos(theta[5]);
    float s5 = std::sin(theta[5]);
    Eigen::Matrix4f T_right_roll;
    T_right_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, c5, s5, 0.0,
                0.0f, -s5, c5, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f right_pitch;
    right_pitch << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, -0.1042f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c6 = std::cos(theta[6]);
    float s6 = std::sin(theta[6]);
    Eigen::Matrix4f T_right_pitch;
    T_right_pitch << c6, 0.0f, s6, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                -s6, 0.0f, c6, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f right_slide;
    right_slide << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, -0.7f,
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f T_right_slide;
    T_right_slide << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, theta[7],
                0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f right_foot_pitch;
    right_foot_pitch << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c8 = std::cos(theta[8]);
    float s8 = std::sin(theta[8]);
    Eigen::Matrix4f T_right_foot_pitch;
    T_right_foot_pitch << c8, 0.0f, s8, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                -s8, 0.0f, c8, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;
        
    Eigen::Matrix4f right_foot_roll;
    right_foot_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;

    float c9 = std::cos(-theta[9]);
    float s9 = std::sin(-theta[9]);
    Eigen::Matrix4f T_right_foot_roll;
    T_right_foot_roll << 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, c9, s9, 0.0,
                0.0f, -s9, c9, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f;


    // left leg
    Eigen::Matrix4f trans_ll;
    trans_ll = left_roll * T_left_roll * left_pitch * T_left_pitch * left_slide * T_left_slide;
    leg.block(4,2,4,1).setConstant(0.6-theta[2]);
    Eigen::Matrix<float, 16, 4> output_ll =  leg * trans_ll.transpose();

    // left foot
    Eigen::Matrix4f trans_lf;
    trans_lf = trans_ll * left_foot_pitch * T_left_foot_pitch * left_foot_roll * T_left_foot_roll;
    Eigen::Matrix<float, 8, 4> output_lf =  foot * trans_lf.transpose();

    // right leg
    Eigen::Matrix4f trans_rl;
    trans_rl = right_roll * T_right_roll * right_pitch * T_right_pitch * right_slide * T_right_slide;
    leg.block(4,2,4,1).setConstant(0.6-theta[7]);
    Eigen::Matrix<float, 16, 4> output_rl =  leg * trans_rl.transpose();

    // right foot
    Eigen::Matrix4f trans_rf;
    trans_rf = trans_rl * right_foot_pitch * T_right_foot_pitch * right_foot_roll * T_right_foot_roll;
    Eigen::Matrix<float, 8, 4> output_rf =  foot * trans_rf.transpose();


    
    // camera info
    Eigen::Matrix<float, 3, 4>  extrinsic;
    extrinsic << 0.7071f, 0.0f, -0.7071f, -0.0141f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.7071, 0, 0.7071, -0.1556;

    Eigen::Matrix<float, 3, 3>  intrinsic;
    intrinsic << 381.3624f, 0.0f, 320.5f,
                0.0f, 381.3624f, 240.5f,
                0.0f, 0.0f, 1.0f;
    
    Eigen::Matrix<float, 3, 3>  R;
    R << 0.0f, 0.0f, 1.0f,
                -1.0f, 0.0f, 0.0f,
                0.0f, -1.0f, 0.0f;

    // left leg in camera frame
    Eigen::Matrix<float, 16, 3> camera_coords_ll = output_ll * extrinsic.transpose() * R;
    Eigen::Matrix<float, 16, 3> raw_pixels_ll = camera_coords_ll * intrinsic.transpose();
    for (int i = 0; i < raw_pixels_ll.rows(); ++i) {
        if (raw_pixels_ll(i, raw_pixels_ll.cols() - 1) < 0){
            raw_pixels_ll(i, raw_pixels_ll.cols() - 1) = 0.01;
        }
    }
    raw_pixels_ll.col(0) = raw_pixels_ll.col(0).array() / raw_pixels_ll.col(2).array();
    raw_pixels_ll.col(1) = raw_pixels_ll.col(1).array() / raw_pixels_ll.col(2).array();
    Eigen::Matrix<int, 16, 3> int_pixels_ll = raw_pixels_ll.array().floor().matrix().cast<int>();

    // left foot in camera frame
    Eigen::Matrix<float, 8, 3> camera_coords_lf = output_lf * extrinsic.transpose() * R;
    Eigen::Matrix<float, 8, 3> raw_pixels_lf = camera_coords_lf * intrinsic.transpose();
    for (int i = 0; i < raw_pixels_lf.rows(); ++i) {
        if (raw_pixels_lf(i, raw_pixels_lf.cols() - 1) < 0){
            raw_pixels_lf(i, raw_pixels_lf.cols() - 1) = 0.01;
        }
    }
    raw_pixels_lf.col(0) = raw_pixels_lf.col(0).array() / raw_pixels_lf.col(2).array();
    raw_pixels_lf.col(1) = raw_pixels_lf.col(1).array() / raw_pixels_lf.col(2).array();
    Eigen::Matrix<int, 8, 3> int_pixels_lf = raw_pixels_lf.array().floor().matrix().cast<int>();

    // right leg in camera frame
    Eigen::Matrix<float, 16, 3> camera_coords_rl = output_rl * extrinsic.transpose() * R;
    Eigen::Matrix<float, 16, 3> raw_pixels_rl = camera_coords_rl * intrinsic.transpose();
    for (int i = 0; i < raw_pixels_rl.rows(); ++i) {
        if (raw_pixels_rl(i, raw_pixels_rl.cols() - 1) < 0){
            raw_pixels_rl(i, raw_pixels_rl.cols() - 1) = 0.01;
        }
    }
    raw_pixels_rl.col(0) = raw_pixels_rl.col(0).array() / raw_pixels_rl.col(2).array();
    raw_pixels_rl.col(1) = raw_pixels_rl.col(1).array() / raw_pixels_rl.col(2).array();
    Eigen::Matrix<int, 16, 3> int_pixels_rl = raw_pixels_rl.array().floor().matrix().cast<int>();

    // right foot in camera frame
    Eigen::Matrix<float, 8, 3> camera_coords_rf = output_rf * extrinsic.transpose() * R;
    Eigen::Matrix<float, 8, 3> raw_pixels_rf = camera_coords_rf * intrinsic.transpose();
    for (int i = 0; i < raw_pixels_rf.rows(); ++i) {
        if (raw_pixels_rf(i, raw_pixels_rf.cols() - 1) < 0){
            raw_pixels_rf(i, raw_pixels_rf.cols() - 1) = 0.01;
        }
    }
    raw_pixels_rf.col(0) = raw_pixels_rf.col(0).array() / raw_pixels_rf.col(2).array();
    raw_pixels_rf.col(1) = raw_pixels_rf.col(1).array() / raw_pixels_rf.col(2).array();
    Eigen::Matrix<int, 8, 3> int_pixels_rf = raw_pixels_rf.array().floor().matrix().cast<int>();

    std::vector<Point> clipping_polygon = {
        {0, 0},    // Clipping polygon vertex 0
        {640, 0},    // Clipping polygon vertex 1
        {640, 480},    // Clipping polygon vertex 2
        {0, 480},    // Clipping polygon vertex 3
        {0, 0}     // Closing point
    };

    Eigen::Matrix<int, 7, 4> surf_vertex_index_ll;
    surf_vertex_index_ll << 0, 1, 5, 4,
                1, 2, 6, 5,
                0, 3, 7, 4,
                8, 9, 13, 12,
                9, 10, 14, 13,
                8, 11, 15, 12,
                12, 13, 14, 15;

    //std::cout << int_pixels_ll << std::endl;

    Eigen::Matrix<int, 3, 4> surf_vertex_index_lf;
    surf_vertex_index_lf << 0, 1, 5, 4,
                3, 2, 6, 7,
                0, 1, 2, 3;

    Eigen::Matrix<int, 7, 4> surf_vertex_index_rl;
    surf_vertex_index_rl << 3, 2, 6, 7,
                1, 2, 6, 5,
                0, 3, 7, 4,
                11, 10, 14, 15,
                9, 10, 14, 13,
                8, 11, 15, 12,
                12, 13, 14, 15;

    Eigen::Matrix<int, 3, 4> surf_vertex_index_rf;
    surf_vertex_index_rf << 0, 1, 5, 4,
                3, 2, 6, 7,
                4, 5, 6, 7;


    // draw mask
    mask = cv::Mat::zeros(480, 640, CV_8UC1);
    draw_mask(clipping_polygon, int_pixels_ll, surf_vertex_index_ll);
    draw_mask(clipping_polygon, int_pixels_lf, surf_vertex_index_lf);
    draw_mask(clipping_polygon, int_pixels_rl, surf_vertex_index_rl);
    draw_mask(clipping_polygon, int_pixels_rf, surf_vertex_index_rf);


    // for(int i = 0; i < surf_vertex_index_ll.rows(); ++i)
    // {   
    //     std::cout << surf_vertex_index_ll.row(i) << std::endl;
    //     std::cout << int_pixels_ll(surf_vertex_index_ll(i,0),0) <<std::endl;
    //     std::cout << int_pixels_ll(surf_vertex_index_ll(i,1),0) <<std::endl;
    //     std::cout << int_pixels_ll(surf_vertex_index_ll(i,2),0) <<std::endl;
        

    //     if (int_pixels_ll(surf_vertex_index_ll(i,0),1)==int_pixels_ll(surf_vertex_index_ll(i,2),1)){
    //         continue;
    //     }

    //     if ((int_pixels_ll(surf_vertex_index_ll(i,0),0)<0 || int_pixels_ll(surf_vertex_index_ll(i,0),1)<0) &&
    //         (int_pixels_ll(surf_vertex_index_ll(i,1),0)<0 || int_pixels_ll(surf_vertex_index_ll(i,1),1)<0) &&
    //         (int_pixels_ll(surf_vertex_index_ll(i,2),0)<0 || int_pixels_ll(surf_vertex_index_ll(i,2),1)<0) &&
    //         (int_pixels_ll(surf_vertex_index_ll(i,3),0)<0 || int_pixels_ll(surf_vertex_index_ll(i,3),1)<0))
    //     {   
    //         continue;
    //     }

        

    //     std::vector<Point> subject_polygon = {
    //         {int_pixels_ll(surf_vertex_index_ll(i,0),0), int_pixels_ll(surf_vertex_index_ll(i,0),1)},    // Subject polygon vertex 0
    //         {int_pixels_ll(surf_vertex_index_ll(i,1),0), int_pixels_ll(surf_vertex_index_ll(i,1),1)},    // Subject polygon vertex 1
    //         {int_pixels_ll(surf_vertex_index_ll(i,2),0), int_pixels_ll(surf_vertex_index_ll(i,2),1)},    // Subject polygon vertex 2
    //         {int_pixels_ll(surf_vertex_index_ll(i,3),0), int_pixels_ll(surf_vertex_index_ll(i,3),1)},
    //         {int_pixels_ll(surf_vertex_index_ll(i,0),0), int_pixels_ll(surf_vertex_index_ll(i,0),1)}     // Closing point
    //     };

    //     Polygon clipping_poly, subject_poly;
    //     bg::assign_points(clipping_poly, clipping_polygon);
    //     bg::correct(clipping_poly); // Fix the orientation of the clipping polygon
    //     bg::assign_points(subject_poly, subject_polygon);
    //     bg::correct(subject_poly); // Fix the orientation of the subject polygon

    //     // Perform Sutherland-Hodgman polygon clipping
    //     std::vector<Polygon> output;
    //     bg::intersection(clipping_poly, subject_poly, output);

    //     //std::cout << output << std::endl;

    //     size_t total_points = 0;
    //     for(const auto& polygon : output) {
    //         total_points += polygon.outer().size();
    //     }

    //     Eigen::MatrixXd Points(total_points, 2);
    //     size_t row = 0;
    //     for(const auto& polygon : output) {
    //         for(const auto& point : polygon.outer()) {
    //             Points(row, 0) = point.x();
    //             Points(row, 1) = point.y();
    //             ++row;
    //         }
    //     }

    //     std::vector<cv::Point> pts;
    //     for(int i = 0; i < Points.rows(); ++i)
    //     {
    //         pts.push_back(cv::Point(Points(i, 0), Points(i, 1)));
    //     }

    //     std::cout << pts << std::endl;

    //     std::vector<std::vector<cv::Point>> pts_all;
    //     pts_all.push_back(pts);
    //     cv::fillPoly(mask, pts_all, cv::Scalar(255));

    // }

    //cv::imshow("Image", mask);
    //cv::waitKey(0);

    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    auto stop = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = 1000*(stop - start);
    std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

    ros::init(argc, argv, "mask_publisher");
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
