#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pwd.h>
void imCb(const sensor_msgs::ImageConstPtr im_msg);

std::string topic_name;
std::string file_path;
bool awaiting_image;
int main(int argc, char** argv)
{
    topic_name = "occupancy_image_color";
    if(argc == 2)
    {
        file_path = argv[1];
    }
    else {
        file_path += "/tmp/occupancy_image";
        ROS_WARN("No path specified defaulting to: %s",file_path.c_str());
    }
    file_path += ".png";
    ros::init(argc, argv, "occupancy_image_publisher");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber im_sub = it.subscribe(topic_name,1,imCb);

    awaiting_image = true;
    ROS_INFO("Wainting for image with topic name: %s", topic_name.c_str());
    ros::Rate r(10);
    while (n.ok() && awaiting_image) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
void imCb(const sensor_msgs::ImageConstPtr im_msg)
{
    // Convert image to opencv
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(im_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imwrite(file_path, cv_ptr->image);
    ROS_INFO("Storing image to path: %s", file_path.c_str());
    awaiting_image = false;
}
