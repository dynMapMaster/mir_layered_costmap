#include <cstdio>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>

using namespace std;

#define USAGE "Usage: \n" \
    "  occupancy_im_saver -h\n"\
    "  occupancy_im_saver [-c <to store color map>] [-f file path]"
#define DEFAULT_FILE "/tmp/occupancy_image.png"

void imCb(const sensor_msgs::ImageConstPtr im_msg);

string file_path;
string topic_name = "/occupancy_image";
bool awaiting_image = false;

int main(int argc, char** argv)
{
    file_path = DEFAULT_FILE;
    for(int i=1; i<argc; i++)
    {
        if(!strcmp(argv[i], "-h"))
        {
            puts(USAGE);
            return 0;
        }
        else if(!strcmp(argv[i], "-c"))
        {
            topic_name += "_color";
        }
        else if(!strcmp(argv[i], "-f"))
        {
            if(++i < argc)
                file_path = argv[i];
            else
            {
                puts(USAGE);
                return 1;
            }
        }
        else
        {
            puts(USAGE);
            return 1;
        }
    }

    ros::init(argc, argv, "occupancy_image_saver");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber im_sub = it.subscribe(topic_name,1,imCb);

    awaiting_image = true;
    ROS_INFO("Wainting for image with topic name: %s", topic_name.c_str());
    ros::Rate r(5);
    while (n.ok() && awaiting_image) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

void imCb(const sensor_msgs::ImageConstPtr im_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    // Convert image to opencv
    try
    {
        cv_ptr = cv_bridge::toCvShare(im_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ROS_INFO("Storing image to path: %s", file_path.c_str());
    cv::imwrite(file_path, cv_ptr->image);
    awaiting_image = false;
}
