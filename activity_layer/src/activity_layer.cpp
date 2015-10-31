/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <activity_layer.h>
#include <layered_costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(layered_costmap_2d::ActivityLayer, layered_costmap_2d::Layer)
PLUGINLIB_DECLARE_CLASS(dynamic_map, ActivityLayer, dynamic_map::ActivityLayer,layered_costmap_2d::Layer)
using layered_costmap_2d::NO_INFORMATION;
using layered_costmap_2d::LETHAL_OBSTACLE;
using layered_costmap_2d::FREE_SPACE;

using layered_costmap_2d::ObservationBuffer;
using layered_costmap_2d::Observation;
using namespace layered_costmap_2d;
namespace dynamic_map
{

void ActivityLayer::onInitialize()
{
    ROS_INFO("Initializing activity map");
    Costmap2D* master = layered_costmap_->getCostmap();
    _map = new activityMap(master->getSizeInCellsX(),master->getSizeInCellsY());
    _xSize = master->getSizeInCellsX();
    _ySize = master->getSizeInCellsY();
    _cellsPerMeter = master->getResolution();
}

void ActivityLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{

}

ActivityLayer::~ActivityLayer()
{

    if (dsrv_)
        delete dsrv_;

}
void ActivityLayer::reconfigureCB(layered_costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{

}

void ActivityLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{

}

void ActivityLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{

}

void ActivityLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)
{

}

void ActivityLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{

}

void ActivityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
    int map_size = 2;
    *min_x = robot_x - map_size;
    *min_y = robot_y - map_size;
    *max_x = robot_x + map_size;
    *max_y = robot_y + map_size;
    ROS_INFO("Updating bound from activity layer");
/*
    int layerMinX, layerMaxX, layerMinY, layerMaxY;
    if(_map->getEditLimits(layerMinX,layerMaxX,layerMinY,layerMaxY))
    {
        double min_xTemp = layerMinX / _cellsPerMeter;
        double max_xTemp = layerMaxX / _cellsPerMeter;
        double min_yTemp = layerMinY / _cellsPerMeter;
        double max_yTemp = layerMaxY / _cellsPerMeter;
        *min_x = (min_xTemp < *min_x) ? min_xTemp : *min_x;
        *max_x = (max_xTemp > *max_x) ? max_xTemp : *max_x;
        *min_y = (min_yTemp < *min_y) ? min_yTemp : *min_y;
        *max_y = (max_yTemp > *max_y) ? max_yTemp : *max_y;
    }
    */
}

void ActivityLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{

}

void ActivityLayer::updateCosts(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    ROS_INFO("Updating cost from activity layer %i, %i, %i, %i",min_i, min_j, max_i, max_j);
    unsigned char* master_array = master_grid.getCharMap();
    unsigned int val = 0;
    int spanSize = _map->getXSize();
    unsigned int span = master_grid.getSizeInCellsX();
    for (int j = min_j; j < max_j; j++)
    {
        unsigned int it = j * span + min_i;
        for (int i = min_i; i < max_i; i++)
        {
            if (!_map->getCellValue(i,j,lastKnownObservation,val)){
            //continue;
            }
           ROS_INFO_ONCE("master_array[%i]=%i",it, master_array[it]);
          unsigned char old_cost = master_array[it];
          //if (old_cost == NO_INFORMATION || old_cost < val)
            master_array[it] = FREE_SPACE;//val;
            it++;
        }
    }
}

void ActivityLayer::addStaticObservation(layered_costmap_2d::Observation& obs, bool marking, bool clearing)
{
}

void ActivityLayer::clearStaticObservations(bool marking, bool clearing)
{

}

bool ActivityLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{

  bool current = true;
  // get the marking observations

  return current;

}

bool ActivityLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{

  bool current = true;

  return current;
}

void ActivityLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{

}

void ActivityLayer::activate()
{
  enabled_ = true;
}

void ActivityLayer::deactivate()
{
  enabled_ = false;
}

void ActivityLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{

}

void ActivityLayer::reset()
{

}

}  // namespace layered_costmap_2d
