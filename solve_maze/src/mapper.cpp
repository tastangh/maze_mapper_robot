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

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    tf::StampedTransform transform;
    try {
        listener->lookupTransform("odom", "base_scan", ros::Time(0), transform);
        
        for(size_t i = 0; i < scan_msg->ranges.size(); i++){
            float range = scan_msg->ranges[i];
            if(std::isfinite(range)) {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

                float x_bs = range * cos(angle);
                float y_bs = range * sin(angle);

                tf::Vector3 point_bs(x_bs, y_bs, 0.0);
                tf::Vector3 point_odom = transform * point_bs;

                int x_pixel = (width / 2) + (int)(point_odom.x() / resolution);
                int y_pixel = (height / 2) - (int)(point_odom.y() / resolution);

                if(x_pixel >= 0 && x_pixel < width && y_pixel >= 0 && y_pixel < height)
                    map_image.at<cv::Vec3b>(y_pixel, x_pixel) = cv::Vec3b(0,0,0);
            }
        }

        cv::imshow("Harita", map_image);
        cv::waitKey(1);
    }
    catch(tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_mapper");
    ros::NodeHandle nh;

    map_image = cv::Mat(height, width, CV_8UC3, cv::Scalar(255,255,255));

    tf::TransformListener tf_listener;
    listener = &tf_listener;

    scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}
