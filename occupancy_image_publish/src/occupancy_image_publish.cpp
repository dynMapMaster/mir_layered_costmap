#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
//using namespace cv;
void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg);
cv::Vec3b getColorFromMapValue(int8_t val);
image_transport::Publisher image_pub;
image_transport::Publisher color_image_pub;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_image_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid::Ptr>("/dynamic_map", 1, &occupancyGridCb);
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("occupancy_image", 1);
    color_image_pub = it.advertise("occupancy_image_color", 1);
    ros::spin();
    return 0;
}

void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg)
{    
    ROS_INFO_NAMED("occupancy_image_publisher","recieved map");
    // Greyscale image
    cv::Mat im(msg->info.height, msg->info.width, CV_8UC1);
    size_t reverse_i = msg->info.height;
    for (size_t i = 0; i < msg->info.height; ++i) {
        size_t row = (--reverse_i)*msg->info.width;
        for (size_t j = 0; j < msg->info.width; ++j) {
            im.at<u_char>(i,j) = msg->data[row + j];
        }
    }
    sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", im).toImageMsg();
    image_pub.publish(im_msg);

    // Color image
    cv::Mat imgColor;//(msg->info.height,msg->info.width, CV_8UC3);
    cv::applyColorMap(im, imgColor, cv::COLORMAP_HOT); // http://docs.opencv.org/2.4/modules/contrib/doc/facerec/colormaps.html

    sensor_msgs::ImagePtr imColor_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgColor).toImageMsg();
    color_image_pub.publish(imColor_msg);
}
