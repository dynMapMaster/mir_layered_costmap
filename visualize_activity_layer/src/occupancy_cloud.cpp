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
uint8_t reverse_cost_translation_table_[256];
ros::Publisher pub;
void HSVtoRGB(float& fR, float& fG, float& fB, const float& fH, const float& fS, const float& fV);
uint8_t getRawCost(char translated_cost);
void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg);
float map(float value, float istart, float istop, float ostart, float ostop) ;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_2d_cloud");
  ros::NodeHandle n;
  // fill cost translation table
  reverse_cost_translation_table_[0] = 0;  // NO obstacle
  reverse_cost_translation_table_[99] = 253;  // INSCRIBED obstacle
  reverse_cost_translation_table_[100] = 254;  // LETHAL obstacle
  reverse_cost_translation_table_[-1] = 255;  // UNKNOWN
  // regular cost values scale the range 1 to 252 (inclusive) to fit into 1 to 98 (inclusive).
  ROS_INFO("cost map");
  for (int i = 1; i < 98; i++)
  {
    reverse_cost_translation_table_[ i ] = char(1 + (251 * (i - 1)) / 97);
    ROS_INFO("%i => %i",i, reverse_cost_translation_table_[ i ]);
  }
  // special values:
  ROS_DEBUG("Startup");
  ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid::Ptr>("/costmap/costmap/costmap", 1, &occupancyGridCb);
  pub = n.advertise<sensor_msgs::PointCloud>("activity_marked_cloud", 2);
  ros::spin();
}

void occupancyGridCb(const nav_msgs::OccupancyGrid::Ptr msg)
{
    if(msg->data.empty())
    {
        ROS_ERROR("Received empty voxel grid");
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
                uint8_t cost_val = getRawCost(msg->data[i]);
                const float value = map(cost_val, 0, 255, 0.3, 0.7);
                const float hue = (cost_val / 255.0) * 360;
                /*
                if(msg->data[i] > 0)
                {
                    ROS_INFO("cost val= %i", cost_val);
                    ROS_INFO("hue = %f", hue);
                }
                */
                float fr(0), fg(0), fb(0);
                HSVtoRGB(fr, fg, fb, hue, 0.5, value);
                uint32_t r = fr * 255.0;
                uint32_t g = fg * 255.0;
                uint32_t b = fb * 255.0;
                col = (r << 16) | (g << 8) | b;
            }

            chan.values[i] = *reinterpret_cast<float*>(&col);
            i++;
        }
    }
    pub.publish(cloud);
}

float map(float value, float istart, float istop, float ostart, float ostop)
{
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

uint8_t getRawCost(char translated_cost)
{
    if(translated_cost < 0)
    {
        return 0;
    }
    else
    {
        return reverse_cost_translation_table_[translated_cost];
    }
}

/*! \brief Convert HSV to RGB color space

  Converts a given set of HSV values `h', `s', `v' into RGB
  coordinates. The output RGB values are in the range [0, 1], and
  the input HSV values are in the ranges h = [0, 360], and s, v =
  [0, 1], respectively.

  \param fR Red component, used as output, range: [0, 1]
  \param fG Green component, used as output, range: [0, 1]
  \param fB Blue component, used as output, range: [0, 1]
  \param fH Hue component, used as input, range: [0, 360]
  \param fS Hue component, used as input, range: [0, 1]
  \param fV Hue component, used as input, range: [0, 1]

*/
void HSVtoRGB(float& fR, float& fG, float& fB, const float& fH, const float& fS, const float& fV) {
  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;

  if(0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if(1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if(2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if(3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if(4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if(5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }

  fR += fM;
  fG += fM;
  fB += fM;
}
