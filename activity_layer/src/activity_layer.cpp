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
using namespace std;
namespace dynamic_map
{

void ActivityLayer::onInitialize()
{
    ROS_INFO("Initializing activity map");
    ros::NodeHandle nh("~/" + name_), g_nh;

    // subscribe to laser scan topics




    _global_frame = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    // get the topics that we'll subscribe to from the parameter server
    nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

    // get our tf prefix
    ros::NodeHandle prefix_nh;
    const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
      ros::NodeHandle source_node(nh, source);

      // get the parameters for the specific topic
      double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
      std::string topic, sensor_frame, data_type;
      bool inf_is_valid, clearing, marking;

      source_node.param("topic", topic, source);
      source_node.param("sensor_frame", sensor_frame, std::string(""));
      source_node.param("observation_persistence", observation_keep_time, 0.0);
      source_node.param("expected_update_rate", expected_update_rate, 0.0);
      source_node.param("data_type", data_type, std::string("LaserScan"));
      source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
      source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
      source_node.param("inf_is_valid", inf_is_valid, false);
      source_node.param("clearing", clearing, false);
      source_node.param("marking", marking, true);

      if (!sensor_frame.empty())
      {
        sensor_frame = tf::resolve(tf_prefix, sensor_frame);
      }

      if (!(data_type == "LaserScan"))
      {
        ROS_FATAL("Only topics that use laser scans are currently supported in activity layer");
        throw std::runtime_error("Only topics that use laser scans are currently supported in activity layer");
      }

      std::string raytrace_range_param_name, obstacle_range_param_name;

      // get the obstacle range for the sensor
      double obstacle_range = 2.5;
      if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
      {
        source_node.getParam(obstacle_range_param_name, obstacle_range);
      }

      // get the raytrace range for the sensor
      double raytrace_range = 3.0;
      if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
      {
        source_node.getParam(raytrace_range_param_name, raytrace_range);
      }

      ROS_INFO("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
                sensor_frame.c_str());
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, _global_frame,
                                     sensor_frame, transform_tolerance)));
    ROS_INFO(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), _global_frame.c_str(), expected_update_rate, observation_keep_time);
        ROS_ERROR("HALLOOOOOOO");
        // create a callback for the topic
        if (data_type == "LaserScan")
        {
          boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
              > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

          boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
              > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, _global_frame, 50));

          if (inf_is_valid)
          {
            filter->registerCallback(
                boost::bind(&ActivityLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
          }
          else
          {
            filter->registerCallback(
                boost::bind(&ActivityLayer::laserScanCallback, this, _1, observation_buffers_.back()));
          }

          observation_subscribers_.push_back(sub);
          observation_notifiers_.push_back(filter);

          observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
        }

        if (sensor_frame != "")
        {
          std::vector < std::string > target_frames;
          target_frames.push_back(_global_frame);
          target_frames.push_back(sensor_frame);
          observation_notifiers_.back()->setTargetFrames(target_frames);
        }

    }
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
    ROS_INFO("Laser scan cb no inf");
    // project the laser into a point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header = message->header;

    // project the scan into a point cloud
    try
    {
      projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", _global_frame.c_str(),
               ex.what());
      projector_.projectLaser(*message, cloud);
    }

    // buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(cloud);
    buffer->unlock();
}

void ActivityLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
    ROS_INFO("Laser scan cb with inf");
    // Filter positive infinities ("Inf"s) to max_range.
    float epsilon = 0.0001;  // a tenth of a millimeter
    sensor_msgs::LaserScan message = *raw_message;
    for (size_t i = 0; i < message.ranges.size(); i++)
    {
      float range = message.ranges[ i ];
      if (!std::isfinite(range) && range > 0)
      {
        message.ranges[ i ] = message.range_max - epsilon;
      }
    }

    // project the laser into a point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header = message.header;

    // project the scan into a point cloud
    try
    {
      projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
               _global_frame.c_str(), ex.what());
      projector_.projectLaser(message, cloud);
    }

    // buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(cloud);
    buffer->unlock();
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
    ROS_INFO("Updating bounds..");
    for(int i = 0; i < observation_buffers_.size(); i++)
    {
        ROS_INFO("Sensor: %i", i);
        vector<Observation> buffer;
        observation_buffers_[i]->getObservations(buffer);
        for(int o = 0; o < buffer.size(); o++)
        {
            //ROS_INFO("observation: %i",o);
            raytrace(buffer[o]);
        }
    }

    //ROS_INFO("Updating bound from activity layer");
    int layerMinX, layerMaxX, layerMinY, layerMaxY;
    if(_map->getEditLimits(layerMinX,layerMaxX,layerMinY,layerMaxY))
    {
        double min_xTemp = layerMinX * _resolution;
        double max_xTemp = layerMaxX * _resolution;
        double min_yTemp = layerMinY * _resolution;
        double max_yTemp = layerMaxY * _resolution;
        *min_x = (min_xTemp < *min_x) ? min_xTemp : *min_x;
        *max_x = (max_xTemp > *max_x) ? max_xTemp : *max_x;
        *min_y = (min_yTemp < *min_y) ? min_yTemp : *min_y;
        *max_y = (max_yTemp > *max_y) ? max_yTemp : *max_y;
        ROS_INFO("Bounds: %f %f %f %f %f",min_xTemp,max_xTemp, min_yTemp, max_yTemp, _resolution);
    }    
    ROS_INFO("Bounds: %f %f %f %f %f",*min_x,*max_x, *min_y, *max_y, _resolution);

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
                continue;
            }
          //ROS_INFO_ONCE("master_array[%i]=%i",it, master_array[it]);
          unsigned char old_cost = master_array[it];
         // if (old_cost == NO_INFORMATION || old_cost < val)
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

void ActivityLayer::raytrace(const Observation& observation)
{
    for(int i = 0; i < observation.cloud_->size();i++){
        //ROS_INFO("Point: %i", i);
        _map->traceLine(observation.origin_.x,observation.origin_.y,observation.cloud_->points[i].x, observation.cloud_->points[i].y);
    }
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

void ActivityLayer::matchSize()
{
    ROS_INFO("Match size called");
    Costmap2D* master = layered_costmap_->getCostmap();

    _xSize = master->getSizeInCellsX();
    _ySize = master->getSizeInCellsY();
    _resolution = maroscster->getResolution();

    _map = new activityMap(_xSize,_ySize,false);
    _tempMap = new activityMap(_xSize, _ySize,false);

}

void ActivityLayer::reset()
{

}

}  // namespace layered_costmap_2d
