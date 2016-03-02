/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: GonÃ§alo Cabrita on 12/06/2013
*********************************************************************/

#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

#define EPSILON 0.0174532925

ros::Publisher range_pub;
void laserScanCallback(const sensor_msgs::LaserScanConstPtr& raw_message)
{

    sensor_msgs::Range range_msgs;
    range_msgs.header = raw_message->header;
    range_msgs.max_range = raw_message->range_max;
    range_msgs.min_range = raw_message->range_min;
    range_msgs.field_of_view = raw_message->angle_max - raw_message->angle_min;
    range_msgs.radiation_type = sensor_msgs::Range::ULTRASOUND;

    // Filter positive infinities ("Inf"s) to max_range.
    float min_range = 1e100;
    sensor_msgs::LaserScan message = *raw_message;
    for (size_t i = 0; i < message.ranges.size(); i++)
    {
      float range = message.ranges[ i ];
      if (range < min_range)
      {
          min_range = range;
      }
    }
    range_msgs.range = min_range;
    range_pub.publish(range_msgs);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_to_laser");

    ROS_INFO("Range2Laser for ROS v0.1");

    ros::NodeHandle n;

    range_pub = n.advertise<sensor_msgs::Range>("range_scan", 10);
    ros::Subscriber laser_sub = n.subscribe("base_scan0",100, &laserScanCallback);
    ros::spin();

    return 0;
}

// EOF
