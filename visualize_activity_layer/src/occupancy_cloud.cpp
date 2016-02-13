/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <layered_costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
using namespace layered_costmap_2d;
ros::Publisher pub;
void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg);
uint8_t map(uint8_t value, uint8_t istart, uint8_t istop, uint8_t ostart, uint8_t ostop);

const float fr[] = { 0, 0.03968253968253968, 0.07936507936507936, 0.119047619047619, 0.1587301587301587, 0.1984126984126984, 0.2380952380952381, 0.2777777777777778, 0.3174603174603174, 0.3571428571428571, 0.3968253968253968, 0.4365079365079365, 0.4761904761904762, 0.5158730158730158, 0.5555555555555556, 0.5952380952380952, 0.6349206349206349, 0.6746031746031745, 0.7142857142857142, 0.753968253968254, 0.7936507936507936, 0.8333333333333333, 0.873015873015873, 0.9126984126984127, 0.9523809523809523, 0.992063492063492, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
const float fg[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.03174603174603163, 0.0714285714285714, 0.1111111111111112, 0.1507936507936507, 0.1904761904761905, 0.23015873015873, 0.2698412698412698, 0.3095238095238093, 0.3492063492063491, 0.3888888888888888, 0.4285714285714284, 0.4682539682539679, 0.5079365079365079, 0.5476190476190477, 0.5873015873015872, 0.6269841269841268, 0.6666666666666665, 0.7063492063492065, 0.746031746031746, 0.7857142857142856, 0.8253968253968254, 0.8650793650793651, 0.9047619047619047, 0.9444444444444442, 0.984126984126984, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
const float fb[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.04761904761904745, 0.1269841269841265, 0.2063492063492056, 0.2857142857142856, 0.3650793650793656, 0.4444444444444446, 0.5238095238095237, 0.6031746031746028, 0.6825396825396828, 0.7619047619047619, 0.8412698412698409, 0.92063492063492, 1};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_2d_cloud");
  ros::NodeHandle n;
  // special values:
  ROS_DEBUG("Startup");
  ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid::Ptr>("/dynamic_map", 1, &occupancyGridCb);
  pub = n.advertise<sensor_msgs::PointCloud>("activity_marked_cloud", 2);
  ros::spin();
}

void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg)
{
    if(msg->data.empty())
    {
        ROS_ERROR("Received empty grid");
        return;
    }
    Costmap2D costmap(msg->info.width, msg->info.height,msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(msg->data.size());
    cloud.channels.resize(1);
    cloud.channels[0].values.resize(msg->data.size());
    cloud.channels[0].name = "rgb";
    cloud.header.frame_id = msg->header.frame_id;
    cloud.header.stamp = ros::Time::now();
    sensor_msgs::ChannelFloat32& chan = cloud.channels[0];
    uint32_t i = 0;
    for (uint32_t y_grid = 0; y_grid < msg->info.height; ++y_grid) {
        for (uint32_t x_grid = 0; x_grid < msg->info.width; ++x_grid) {
            geometry_msgs::Point32& p = cloud.points[i];
            double wx, wy;
            costmap.mapToWorld(x_grid, y_grid, wx, wy); // not sure why wy and wx needs to be switched
            p.x = wx; p.y = wy;
            p.z = 0;
            //ROS_INFO("val in map=%i",msg->data[i] );
            uint32_t col;
            if(msg->data[i] == -1) // unknown cell
            {
                col = UINT_MAX;
            }
            else
            {                
                uint8_t cost_val = (uint8_t)(msg->data[i]);
                cost_val = map(cost_val, 0, 255, 0, 63);
                ROS_INFO("val map=%i",cost_val );
                ROS_INFO("red=%f",fr[cost_val] );
                uint32_t r = fr[cost_val] * 255.0;
                uint32_t g = fg[cost_val] * 255.0;
                uint32_t b = fb[cost_val] * 255.0;
                col = (r << 16) | (g << 8) | b;
            }

            chan.values[i] = *reinterpret_cast<float*>(&col);
            i++;
        }
    }
    ROS_INFO("Publishing cloud");
    pub.publish(cloud);
}
uint8_t map(uint8_t value, uint8_t istart, uint8_t istop, uint8_t ostart, uint8_t ostop)
{
    return ostart + double(ostop - ostart) * (double(value - istart) / double(istop - istart)) + 0.5;
}

