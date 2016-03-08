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
#ifndef COSTMAP_2D_TEST_LAYER_H_
#define COSTMAP_2D_TEST_LAYER_H_

#include <ros/ros.h>
#include <layered_costmap_2d/costmap_layer.h>
#include <layered_costmap_2d/layered_costmap.h>
#include <layered_costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <layered_costmap_2d/ObstaclePluginConfig.h>
#include <layered_costmap_2d/footprint.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <observation_interface.h>
#include <costmap_interpretator.h>
// Learners
#include <pmac_learner.h>
#include <bayes_learner.h>
#include <probabilistic_filter.h>

#include "activity_layer/loadDynaicMap.h"
#include "activity_layer/saveDynaicMap.h"

namespace dynamic_map
{

typedef Bayes_learner LearnerT;
typedef Probabilistic_filter FilterT;

class ActivityLayer : public layered_costmap_2d::Layer
{
public:
  ActivityLayer()
  {
    ROS_INFO("creating activity layer");
    _observation_map = NULL;
    _map = NULL; // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~ActivityLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  void matchSize();

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                         const boost::shared_ptr<layered_costmap_2d::ObservationBuffer>& buffer);

   /**
    * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
    * @param message The message returned from a message notifier
    * @param buffer A pointer to the observation buffer to update
    */
  void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message,
                                 const boost::shared_ptr<layered_costmap_2d::ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                          const boost::shared_ptr<layered_costmap_2d::ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud2 messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                           const boost::shared_ptr<layered_costmap_2d::ObservationBuffer>& buffer);

  // for testing purposes
  void addStaticObservation(layered_costmap_2d::Observation& obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  Get the observations used to mark space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getMarkingObservations(std::vector<layered_costmap_2d::Observation>& marking_observations) const;

  /**
   * @brief  Get the observations used to clear space
   * @param clearing_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getClearingObservations(std::vector<layered_costmap_2d::Observation>& clearing_observations) const;

  /**
   * @brief  Clear freespace based on one observation
   * @param clearing_observation The observation used to raytrace
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  virtual void raytrace(const layered_costmap_2d::Observation& observation);

  void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                       double* max_x, double* max_y);

 
  double max_obstacle_height_;  ///< @brief Max Obstacle Height



  bool rolling_window_;
  dynamic_reconfigure::Server<layered_costmap_2d::ObstaclePluginConfig> *dsrv_;

  int combination_method_;

protected:
  laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  std::vector<boost::shared_ptr<layered_costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors

private:
    void reconfigureCB(layered_costmap_2d::ObstaclePluginConfig &config, uint32_t level);    
    FilterT* _observation_map;
    LearnerT* _map;
    int _xSize, _ySize;
    double _resolution;
    static const double SENSOR_STD_DEV = 0 ; //0.01; // in m
    std::string _global_frame;
    double _max_range;


    bool poseIsAccurate;
    ros::Subscriber poseSubscriber;

    static const double POSE_POS_STDDEV = 0.3162; //sqrt(0.1)
    static const double POSE_ORI_STDDEV = 0.1732; // sqrt(0.03)
    double _angle_std_dev;
    double _x_std_dev, _y_std_dev;
    // Pose callback
    void poseCB(geometry_msgs::PoseWithCovarianceStamped pose);

    // Request a map for initialization from mapserver
    nav_msgs::OccupancyGrid requestMap();

    // Determine the initial value from the received map
    Costmap_interpretator::Initial_values determineInitialValue(unsigned char val);

    // counter of laser scans received since last handling
    int laserScanWaitingCounter;

    // Services for saving and loading
    ros::ServiceServer saveService;
    ros::ServiceServer loadService;

    bool saveDynamicMap(activity_layer::saveDynaicMap::Request &req, activity_layer::saveDynaicMap::Response &resp);
    bool loadDynamicMap(activity_layer::loadDynaicMap::Request &req, activity_layer::loadDynaicMap::Response &resp);
};

}  // namespace layered_costmap_2d

#endif  // COSTMAP_2D_OBSTACLE_LAYER_H_
