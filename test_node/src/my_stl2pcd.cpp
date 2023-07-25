#include <iostream>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::polygon<Point> Polygon;

int main()
{
    // Define the clipping polygon
    std::vector<Point> clipping_polygon = {
        {0, 0},    // Clipping polygon vertex 0
        {640, 0},    // Clipping polygon vertex 1
        {640, 480},    // Clipping polygon vertex 2
        {0, 480},    // Clipping polygon vertex 3
        {0, 0}     // Closing point
    };

    // Define the subject polygon to be clipped
    std::vector<Point> subject_polygon = {
        {150, 200},    // Subject polygon vertex 0
        {250, 150},    // Subject polygon vertex 1
        {-100, 200},    // Subject polygon vertex 2
        {150, 200}     // Closing point
    };

    // Create Boost Geometry polygons
    Polygon clipping_poly, subject_poly;
    bg::assign_points(clipping_poly, clipping_polygon);
    bg::correct(clipping_poly); // Fix the orientation of the clipping polygon
    bg::assign_points(subject_poly, subject_polygon);
    bg::correct(subject_poly); // Fix the orientation of the subject polygon

    // Perform Sutherland-Hodgman polygon clipping
    std::vector<Polygon> output;
    bg::intersection(clipping_poly, subject_poly, output);

    std::cout << "The size of the vector is: " << output.size() << std::endl;

    ////
    for(const auto& polygon : output) {
        boost::geometry::for_each_point(polygon, [](const Point& p) {
            std::cout << "Point: " << boost::geometry::wkt(p) << "\n";
        });
        std::cout << std::endl;
    }

    size_t total_points = 0;
    for(const auto& polygon : output) {
        total_points += polygon.outer().size();
    }

    // Create the Eigen matrix to hold the points
    Eigen::MatrixXd Points(total_points, 2);
    
    size_t row = 0;
    for(const auto& polygon : output) {
        for(const auto& point : polygon.outer()) {
            Points(row, 0) = point.x();
            Points(row, 1) = point.y();
            ++row;
        }
    }
    
    std::cout << "Points:\n" << Points << std::endl;

    std::vector<cv::Point> pts;
    for(int i = 0; i < Points.rows(); ++i)
    {
        pts.push_back(cv::Point(Points(i, 0), Points(i, 1)));
    }
    
    cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC1);

    std::vector<std::vector<cv::Point>> pts_all;
    pts_all.push_back(pts);

    cv::fillPoly(img, pts_all, cv::Scalar(255));

    cv::imshow("Image", img);
    cv::waitKey(0);

    return 0;
}
