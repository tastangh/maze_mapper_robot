#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <opencv2/opencv.hpp>

ros::Subscriber odom_sub, scan_sub;
tf::TransformListener *listener;

cv::Mat map_image;
float resolution = 0.05;
int width = 800, height = 800;

nav_msgs::Odometry latest_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    latest_odom = *odom_msg;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    tf::StampedTransform transform_scan_to_footprint, transform_footprint_to_odom;

    try {
        listener->lookupTransform("base_footprint", "base_scan", ros::Time(0), transform_scan_to_footprint);
        listener->lookupTransform("odom", "base_footprint", ros::Time(0), transform_footprint_to_odom);

        cv::Mat updated_map = map_image.clone();

        for(size_t i = 0; i < scan_msg->ranges.size(); i++){
            float range = scan_msg->ranges[i];
            if(std::isfinite(range)) {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

                tf::Vector3 point_bs(range * cos(angle), range * sin(angle), 0.0);

                tf::Vector3 point_bf = transform_scan_to_footprint * point_bs;

                tf::Vector3 point_odom = transform_footprint_to_odom * point_bf;

                int x_pixel = (width / 2) + static_cast<int>(point_odom.x() / resolution);
                int y_pixel = (height / 2) - static_cast<int>(point_odom.y() / resolution);

                if(x_pixel >= 0 && x_pixel < width && y_pixel >= 0 && y_pixel < height)
                    updated_map.at<cv::Vec3b>(y_pixel, x_pixel) = cv::Vec3b(0, 0, 0);
            }
        }

        map_image = updated_map;
        cv::imshow("Harita", map_image);
        cv::waitKey(1);
    }
    catch(tf::TransformException &ex) {
        ROS_WARN("Transform error: %s", ex.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_mapper");
    ros::NodeHandle nh;

    map_image = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    tf::TransformListener tf_listener;
    listener = &tf_listener;

    scan_sub = nh.subscribe("/scan", 10, scanCallback);
    odom_sub = nh.subscribe("/odom", 10, odomCallback);
    
    ROS_INFO("Mapper node started ve connected to ROS.");
    
    ros::spin();
    return 0;
}