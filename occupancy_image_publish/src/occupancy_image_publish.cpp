#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
//using namespace cv;
void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg);
cv::Vec3b getColorFromMapValue(int8_t val);
image_transport::Publisher image_pub;
image_transport::Publisher color_image_pub;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_image_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid::Ptr>("/costmap/costmap/costmap", 1, &occupancyGridCb);
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("occupancy_image", 1);
    color_image_pub = it.advertise("occupancy_image_color", 1);
    ros::spin();
    return 0;
}

void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg)
{    
    // Greyscale image
    cv::Mat im(msg->info.height, msg->info.width, CV_8SC1);
    unsigned im_length = msg->info.height*msg->info.width;
    for (int i = 0; i < msg->info.height; ++i) {
        for (int j = 0; j < msg->info.width; ++j) {
            im.data[i* msg->info.width + j] = msg->data[(msg->info.height-i)*msg->info.width + j];
        }
    }
    sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", im).toImageMsg();
    image_pub.publish(im_msg);



    // Color image
    cv::Mat imgColor(msg->info.height,msg->info.width, CV_8UC3);
    for(int y=0; y < msg->info.height;y++){
        cv::Vec3b* imgRow = imgColor.ptr<cv::Vec3b>(y);
        for(int x = 0; x < msg->info.width;x++){
            imgRow[x] = getColorFromMapValue(msg->data[(msg->info.height-y)*msg->info.width + x]);
        }
    }
    sensor_msgs::ImagePtr imColor_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgColor).toImageMsg();
    color_image_pub.publish(imColor_msg);

}

cv::Vec3b getColorFromMapValue(int8_t val)
{
    cv::Vec3b res;

    if(val < 0)
    {
        res[0] = 128;
        res[1] = 128;
        res[2] = 128;
    }
    else if( val >= 0 && val <=100)
    {
        if(val <= 50){
            res[2] = 250 - (50 -val) * 3;
            res[1] = 250;
            res[0] = 0;
        }
        else
        {
            res[2] = 250;
            res[1] = 250 - val * 2.5;
            res[0] = 0;
           // ROS_INFO("val: %i   - green: %i",val,res[1]);
        }
    }
    else
    {
        res[0] = 0;
        res[1] = 0;
        res[2] = 0;
    }
    return res;
}
