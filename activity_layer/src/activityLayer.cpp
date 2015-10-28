#include <activityLayer.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(activity_layer, activityLayer, activity_layer::activityLayer, layered_costmap_2d::Layer)

using namespace activity_layer;
using namespace layered_costmap_2d;

activityLayer::activityLayer():Layer()
{
    ROS_INFO("creating activity layer");
    _map = NULL;
}

/**
 * @brief This is called by the LayeredCostmap to poll this plugin as to how
 *        much of the costmap it needs to update. Each layer can increase
 *        the size of this bounds.
 *
 * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
 * by Lu et. Al, IROS 2014.
 */
void activityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                          double* max_x, double* max_y)
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

/**
 * @brief Actually update the underlying costmap, only within the bounds
 *        calculated during UpdateBounds().
 */
void activityLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    ROS_INFO("Updating cost from activity layer");
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

          unsigned char old_cost = master_array[it];
          //if (old_cost == NO_INFORMATION || old_cost < val)
            master_array[it] = LETHAL_OBSTACLE;//val;
        }
    }
}



void activityLayer::onInitialize()
{
    ROS_INFO("Initializing activity layer");
    // Subscribe to topics -> see obstacle layer
    /*
    ros::NodeHandle nh("~/" + name_), g_nh;

    // subscribe to laser scan topics


    Costmap2D* master = layered_costmap_->getCostmap();

    _xSize = master->getSizeInCellsX();
    _ySize = master->getSizeInCellsY();
    _cellsPerMeter = master->getResolution();
    _map = new activityMap(_xSize,_ySize,false);
    _tempMap = new activityMap(_xSize, _ySize,false);


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

      ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
                sensor_frame.c_str());
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, _global_frame,
                                     sensor_frame, transform_tolerance)));
    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), _global_frame.c_str(), expected_update_rate, observation_keep_time);

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
                boost::bind(&activityLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
          }
          else
          {
            filter->registerCallback(
                boost::bind(&activityLayer::laserScanCallback, this, _1, observation_buffers_.back()));
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

    */

}

void activityLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<ObservationBuffer>& buffer)
{
    ROS_INFO("ACTIVITY MAP: LASER SCAN (INF VALID) RECEIVED!");
    // project the laser into a point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header = raw_message->header;

    // project the scan into a point cloud
    try
    {
      projector_.transformLaserScanToPointCloud(raw_message->header.frame_id, *raw_message, cloud, *tf_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", _global_frame.c_str(),
               ex.what());
      projector_.projectLaser(*raw_message, cloud);
    }

    // buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(cloud);
    buffer->unlock();
}

void activityLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<ObservationBuffer>& buffer)
{
    ROS_INFO("ACTIVITY MAP: LASER SCAN RECEIVED!");
}

void activityLayer::markObservationOnTempMap()
{

}

void activityLayer::copyTempToActivityMap()
{

}


void activityLayer::activate()
{
    enabled_ = true;
}

void activityLayer::deactivate()
{
    enabled_ = false;
}
