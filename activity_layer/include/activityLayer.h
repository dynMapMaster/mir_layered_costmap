
#ifndef _ACTIVITY_LAYER_H
#define _ACTIVITY_LAYER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <activityMap.hpp>

#include <layered_costmap_2d/layer.h>
#include <layered_costmap_2d/layered_costmap.h>
#include <layered_costmap_2d/observation_buffer.h>
#include <layered_costmap_2d/costmap_2d.h>
// #include <activityLayer/ActivityPluginConfig.h>
#include <cstdlib>

namespace activity_layer
{
class activityLayer : public layered_costmap_2d::Layer
{
public:
    activityLayer();
    ~activityLayer();

    /**
    * @brief This is called by the LayeredCostmap to poll this plugin as to how
    *        much of the costmap it needs to update. Each layer can increase
    *        the size of this bounds.
    *
    * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
    * by Lu et. Al, IROS 2014.
    */
    void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

    /**
    * @brief Actually update the underlying costmap, only within the bounds
    *        calculated during UpdateBounds().
    */
    void updateCosts(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /** @brief Stop publishers. */
    void deactivate();

    /** @brief Restart publishers if they've been stopped. */
    void activate();

    void reset();

    /** @brief LayeredCostmap calls this whenever the footprint there
     * changes (via LayeredCostmap::setFootprint()).  Override to be
     * notified of changes to the robot's footprint. */
    void onFootprintChanged();

protected:
    void onInitialize();
    void laserScanReceived( const sensor_msgs::LaserScanPtr scanReceived);
    void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<layered_costmap_2d::ObservationBuffer>& buffer);
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<layered_costmap_2d::ObservationBuffer>& buffer);
    void markObservationOnTempMap();
    void copyTempToActivityMap();


    int _xSize, _ySize;
    double _cellsPerMeter;

    laser_geometry::LaserProjection projector_;

    std::string _global_frame;

     std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
     std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
     std::vector<boost::shared_ptr<layered_costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors

private:
    activityMap* _map;
    activityMap* _tempMap;

};
}


#endif
