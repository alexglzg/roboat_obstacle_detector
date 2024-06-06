#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

class EllipseDetector {
public:
    EllipseDetector() {
        map_sub_ = nh_.subscribe("planning/obstacle/map", 10, &EllipseDetector::mapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ellipses", 10);
    }

    /*void detect() {
        // No need to implement as it is used only for placeholder in Python
    }*/

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        double x_origin = msg->info.origin.position.x;
        double y_origin = msg->info.origin.position.y;

        std::vector<int8_t> occupancy_data = msg->data;
        cv::Mat image_data = cv::Mat::zeros(height, width, CV_8UC1);

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                if (occupancy_data[i * width + j] == -1) {
                    image_data.at<uint8_t>(height - 1 - i, j) = 127;  // Unknown
                } else if (occupancy_data[i * width + j] == 0) {
                    image_data.at<uint8_t>(height - 1 - i, j) = 0;    // Free space
                } else {
                    image_data.at<uint8_t>(height - 1 - i, j) = 255;  // Occupied space
                }
            }
        }

        cv::Mat bgr_image, grayscale_again, thresh;
        cv::cvtColor(image_data, bgr_image, cv::COLOR_GRAY2BGR);
        cv::cvtColor(bgr_image, grayscale_again, cv::COLOR_BGR2GRAY);
        cv::threshold(grayscale_again, thresh, 252, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        cv::Mat result = bgr_image.clone();
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < contours.size(); ++i) {
            if (contours[i].size() >= 5) {
                cv::RotatedRect ellipse = cv::fitEllipse(contours[i]);
                double p_centx = ellipse.center.x;
                double p_centy = ellipse.center.y;
                double p_width = ellipse.size.width;
                double p_height = ellipse.size.height;
                double d_angle = ellipse.angle;

                double e_centx = x_origin + (p_centx) * resolution;
                double e_centy = y_origin + (height - p_centy) * resolution;
                double e_width = p_width * resolution / 2;
                double e_height = p_height * resolution / 2;
                double e_angle = -d_angle * CV_PI / 180.0;

                if ((e_width >= 1) || (e_height >= 1)){
                    continue;
                }

                visualization_msgs::Marker marker = createEllipseMarker(e_centx, e_centy, e_width, e_height, e_angle, i);
                marker_array.markers.push_back(marker);

                //cv::ellipse(result, ellipse, cv::Scalar(0, 0, 255), 2);
            }
        }

        marker_pub_.publish(marker_array);
    }

    visualization_msgs::Marker createEllipseMarker(double center_x, double center_y, double semi_major_axis, double semi_minor_axis, double orientation, int id, const std::string& frame_id = "map") {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = std::to_string(id);
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;  // Line width
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 0.0;  // Blue

        int num_points = 100;
        double angle_step = 2 * M_PI / num_points;
        for (int i = 0; i <= num_points; ++i) {
            double angle = i * angle_step;
            double x = semi_major_axis * cos(angle);
            double y = semi_minor_axis * sin(angle);

            double x_rot = x * cos(orientation) - y * sin(orientation);
            double y_rot = x * sin(orientation) + y * cos(orientation);

            geometry_msgs::Point point;
            point.x = center_x + x_rot;
            point.y = center_y + y_rot;
            point.z = 0;
            marker.points.push_back(point);
        }

        return marker;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ellipse_detector");
    EllipseDetector ellipseDetector;
    ros::Rate rate(10);

    while (ros::ok()) {
        //ellipseDetector.detect();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}