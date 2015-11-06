#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
//using namespace cv;
void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg);
image_transport::Publisher image_pub;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_image_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid::Ptr>("/robot_0/costmap/costmap/costmap", 1, &occupancyGridCb);
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("occupancy_image", 1);
    ros::spin();
    return 0;
}

void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg)
{    
    cv::Mat im(msg->info.height, msg->info.width, CV_8SC1);
    unsigned im_length = msg->info.height*msg->info.width;
    for (int i = 0; i < msg->info.height; ++i) {
        for (int j = 0; j < msg->info.width; ++j) {
            im.data[i* msg->info.width + j] = msg->data[(msg->info.height-i)*msg->info.width + j];
        }
    }
    sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", im).toImageMsg();
    image_pub.publish(im_msg);
}
