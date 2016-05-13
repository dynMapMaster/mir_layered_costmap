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

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <fstream>
#include <sstream>

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
    _x_std_dev = -1;
    _y_std_dev = -1;

    laserScanWaitingCounter = 0;

    ROS_INFO("Initializing activity map");
    ros::NodeHandle nh("~/" + name_), g_nh;

    // subscribe to laser scan topics
    _global_frame = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    //Subscribe to amcl pose
    string poseTopicName ="";
    nh.param("pose_topic",poseTopicName,string("amcl_pose"));
    poseIsAccurate = false;
    poseSubscriber = g_nh.subscribe(poseTopicName, 1, &ActivityLayer::poseCB,this);

    // Advertise asve and load services
    saveService = g_nh.advertiseService("save_dynamicMap",&ActivityLayer::saveDynamicMap,this);
    loadService = g_nh.advertiseService("load_dynamicMap",&ActivityLayer::loadDynamicMap,this);

    std::string topics_string;
    // get the topics that we'll subscribe to from the parameter server
    nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

    // get our tf prefix
    ros::NodeHandle prefix_nh;
    const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    // start observation map timer
    _observation_map_timer = g_nh.createTimer(ros::Duration(60,0),&ActivityLayer::callback_observation_timer,this);
    tracer_thread_running = false;

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
      source_node.param("observation_persistence", observation_keep_time, 999.0);
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
    _xSize = DBL_MAX;
    _ySize = DBL_MAX;
    matchSize();
}

void ActivityLayer::callback_observation_timer(const ros::TimerEvent&)
{
    if(!tracer_thread_running)
    {
        overtime = false;
        tracer_thread_running = true;
        tracer_thread = std::thread(&ActivityLayer::tracer_thread_function,this);
        tracer_thread.detach();
    }
    else
    {
        ROS_ERROR("UNABLE TO ADD OBSERVATIONS");
        overtime = true;
    }
}

void ActivityLayer::tracer_thread_function()
{
    do
    {
        overtime = false;
        for(size_t i = 0; i < observation_buffers_.size(); i++)
        {
            list<Observation> buffer;

            observation_buffers_[i]->lock();
            observation_buffers_[i]->getObservations(buffer,true);
            observation_buffers_[i]->unlock();            
            list<Observation>::iterator obs_it;
            for (obs_it = buffer.begin(); obs_it != buffer.end(); ++obs_it)
            {
                raytrace(*obs_it);
            }
        }
        _map_mutex.lock();
        _map->addObservationMap(_observation_map);
        _map_mutex.unlock();
        ROS_INFO("ADDING OBSERVATIONS - DONE");
    } while(overtime);
    tracer_thread_running = false;
}


void ActivityLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{

}

ActivityLayer::~ActivityLayer()
{

    if (dsrv_)
        delete dsrv_;
    _map_mutex.lock();
    if(_observation_map)
        delete _observation_map;

    if(_map)
        delete _map;
    _map_mutex.unlock();

}
void ActivityLayer::reconfigureCB(layered_costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{

}
double prev_recive_time;
void ActivityLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{          
        sensor_msgs::LaserScan message = *raw_message;
        ros::Time recive_time = message.header.stamp;
        double now = recive_time.toSec();
        /*
        if( (now - prev_recive_time) > 0.1)
        {
            ROS_WARN("Time between scans: %f",now - prev_recive_time);
        }
        */
        prev_recive_time = now;
        for (size_t i = 0; i < message.ranges.size(); i++)
        {
          float range = message.ranges[ i ];
          if (range >= message.range_max && range > 0)
          {
            _max_range = message.range_max;
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
          ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", _global_frame.c_str(),
                   ex.what());
          projector_.projectLaser(message, cloud);
        }

        // buffer the point cloud
        if(_angle_std_dev < 0 || _x_std_dev < 0 || _y_std_dev < 0)
        {
            ROS_ERROR("BUFFER INPUT - COV = 0");
            return;
        }
        buffer->lock();
        buffer->bufferCloud(cloud,_angle_std_dev,_x_std_dev,_y_std_dev);
        buffer->unlock();
}

void ActivityLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message, const boost::shared_ptr<ObservationBuffer>& buffer)
{
    ROS_ERROR("INF CALLED");
    /*
    if(poseIsAccurate)
    {
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

        laserScanWaitingCounter++;
        if(laserScanWaitingCounter > 1)
        {
            // Waste of time -> consider running in a timer callback
            for(size_t i = 0; i < observation_buffers_.size(); i++)
            {
                vector<Observation> buffer;
                observation_buffers_[i]->lock();
                observation_buffers_[i]->getObservations(buffer);
                observation_buffers_[i]->unlock();
                for(size_t o = 0; o < buffer.size(); o++)
                {
                    //ROS_INFO("observation: %i",o);
                    raytrace(buffer[o]);
                }
            }
            laserScanWaitingCounter = 0;
        }
    }
    else {
        ROS_WARN("Skipping update of dynamic map since robot pose is too inaccurate");
    }
    */
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
    //ROS_INFO("Updating bound from activity layer");
    int layerMinX, layerMaxX, layerMinY, layerMaxY;
    _map_mutex.lock();
    _map->loadUpdateBounds(layerMinX, layerMaxX, layerMinY, layerMaxY);
    _map_mutex.unlock();
    if(layerMinX < 0 || layerMaxX < 0 || layerMinY < 0 || layerMaxY < 0)
    {

    }
    else
    {
        ROS_DEBUG("Activity layer: updateing bounds");
        Costmap2D* master = layered_costmap_->getCostmap();
        double min_xTemp, max_xTemp, min_yTemp, max_yTemp ;
        master->mapToWorld(layerMinX,layerMinY,min_xTemp,min_yTemp);
        master->mapToWorld(layerMaxX,layerMaxY,max_xTemp,max_yTemp);

        *min_x = (min_xTemp < *min_x) ? min_xTemp : *min_x;
        *max_x = (max_xTemp > *max_x) ? max_xTemp : *max_x;
        *min_y = (min_yTemp < *min_y) ? min_yTemp : *min_y;
        *max_y = (max_yTemp > *max_y) ? max_yTemp : *max_y;
        //ROS_INFO("Bounds: %f %f %f %f %f",min_xTemp,max_xTemp, min_yTemp, max_yTemp, _resolution);
    }
    //ROS_INFO("Bounds: %f %f %f %f %f",*min_x,*max_x, *min_y, *max_y, _resolution);

}

void ActivityLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{

}

void ActivityLayer::updateCosts(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    ROS_INFO("Updating cost from activity layer %i, %i, %i, %i",min_i, min_j, max_i, max_j);
    unsigned char* master_array = master_grid.getCharMap();
    unsigned char val = 0;
    unsigned int span = master_grid.getSizeInCellsX();
    for (int j = min_j; j < max_j; j++)
    {
        //ROS_INFO("j=%i",j);
        unsigned int it = j * span + min_i;
        for (int i = min_i; i < max_i; i++)
        {
            _map_mutex.lock();
            if (_map->getCellValue(i,j,val)){
                unsigned char old_cost = master_array[it];
                //if (old_cost == NO_INFORMATION || old_cost < val)
                    master_array[it] =  val;
            }
            else
            {
                master_array[it] = 96;
            }
            _map_mutex.unlock();
            it++;
        }
    }
    _map_mutex.lock();
    _map->resetEditLimits();
    double score = _map->getPredictScore();
    _map_mutex.unlock();
    ROS_INFO("Score: %f    ", score);
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
    Costmap2D* master = layered_costmap_->getCostmap();
    _map_mutex.lock();
    for(size_t i = 0; i < observation.cloud_->size();i++){
        //calculate range
        double range = sqrt(pow(observation.origin_.x - observation.cloud_->points[i].x,2)+pow(observation.origin_.y - observation.cloud_->points[i].y,2));
        bool mark_end = (fabs(range-_max_range) < 0.001 ? false : true);

        int x0,y0, x1, y1;
        master->worldToMapEnforceBounds(observation.origin_.x,observation.origin_.y,x0,y0);
        master->worldToMapEnforceBounds(observation.cloud_->points[i].x,observation.cloud_->points[i].y,x1,y1);
        try
        {
            _observation_map->_angle_std_dev = observation.angle_std_dev_;
            if(observation.x_std_dev_ < 0.0 || observation.y_std_dev_ < 0.0)
            {
                ROS_ERROR("NO POSE RECEIVED - Aborting raytrace");
                return;
            }
            _observation_map->_x_std_dev = observation.x_std_dev_;
            _observation_map->_y_std_dev = observation.y_std_dev_;

            _observation_map->raytrace(x0,y0,x1,y1,mark_end);
        }
        catch(const char* s)
        {
            ROS_ERROR("%s",s);
        }
        catch(...)
        {
            ROS_ERROR("Raytrace unknown error");
        }
    }
    _map_mutex.unlock();
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
    Costmap2D* master = layered_costmap_->getCostmap();
    nav_msgs::OccupancyGrid grid = requestMap();
    master->resizeMap(grid.info.width, grid.info.height, grid.info.resolution,
              grid.info.origin.position.x, grid.info.origin.position.y);
    _map_mutex.lock();
    if(_map)
        delete _map;
    if(_observation_map)
        delete _observation_map;
    int prev_x_size = _xSize, prev_y_size = _ySize;


    _xSize = master->getSizeInCellsX();
    _ySize = master->getSizeInCellsY();
    ROS_INFO("map size from (%i,%i) -> (%i, %i)",prev_x_size,prev_y_size,_xSize,_ySize);
    _resolution = master->getResolution();
    _observation_map = new FilterT(_xSize, _ySize, _resolution, SENSOR_STD_DEV / _resolution, master->getOriginX(), master->getOriginY());
    _map = new LearnerT(_xSize, _ySize, _resolution);
    _map_mutex.unlock();
    // request map from mapserver

    // Initialize _map
    if(prev_x_size >= grid.info.width && prev_y_size >= grid.info.height)
    {
        for(int x = 0; x < grid.info.width; x++){
            for(int y = 0; y < grid.info.height; y++){
                Costmap_interpretator::Initial_values val = determineInitialValue(grid.data[y * grid.info.width + x]);
                u_char learned_value;
                _map_mutex.lock();
                _map->initCell(x,y,val);
                _map_mutex.unlock();
            }
        }
    }
}



nav_msgs::OccupancyGrid ActivityLayer::requestMap()
{
    // get map via RPC
      nav_msgs::GetMap::Request  req;
      nav_msgs::GetMap::Response resp;
      ROS_INFO("Requesting the map...");
      while(!ros::service::call("static_map", req, resp))
      {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
      }
    return resp.map;
}

Costmap_interpretator::Initial_values ActivityLayer::determineInitialValue(unsigned char val)
{
    if(val == 224)
    {
        return Costmap_interpretator::Obstacle;
    }
    else if( val == 96)
    {
        return Costmap_interpretator::Unknown;
    }
    else if( val < 224)
    {
        return Costmap_interpretator::Free;
    }
    else
    {
        ROS_ERROR("Activity layer: Unknown map value received: %i", val);
    }
    return Costmap_interpretator::Unknown;
}

void ActivityLayer::reset()
{

}

void ActivityLayer::poseCB(geometry_msgs::PoseWithCovarianceStamped pose)
{
    _x_std_dev = std::sqrt(pose.pose.covariance[0]);
    _y_std_dev = std::sqrt(pose.pose.covariance[7]);
    _angle_std_dev = std::sqrt(pose.pose.covariance[35]);
    if(pose.pose.covariance[0] < POSE_POS_STDDEV && pose.pose.covariance[7] < POSE_POS_STDDEV && pose.pose.covariance[35] < POSE_ORI_STDDEV)
    {
        poseIsAccurate = true;
    }
    else
    {
        poseIsAccurate = false;
    }
}

bool ActivityLayer::saveDynamicMap(activity_layer::saveDynaicMap::Request &req, activity_layer::saveDynaicMap::Response &resp)
{
    resp.success = false;
    _map_mutex.lock();
    ROS_INFO("Score: %f      ",_map->getPredictScore());
    _map_mutex.unlock();
    try
    {
        _map_mutex.lock();
        std::vector<std::vector<double> > mapSerialized = _map->serialize();
        _map_mutex.unlock();
        // Create an output archive
        std::ofstream ofs;
        ofs.open(req.path.c_str());
        for(int i = 0; i < mapSerialized.size();i++){
            for(int k = 0; k < mapSerialized[i].size();k++){
                if(i == 0)
                {
                    ROS_ERROR("%f",mapSerialized[i][k]);
                }
                // write content
                ofs << mapSerialized[i][k];
                ofs << " ";
            }
            //write newline
            ofs << "\n";
        }
        resp.success = true;
        ROS_INFO("SAVE DYNAMIC MAP - SUCCESS");
    }
    catch(...)
    {
        ROS_ERROR("SAVE DYNAMIC MAP - FAILED!");
    }


    return true;
}

bool ActivityLayer::loadDynamicMap(activity_layer::loadDynaicMap::Request &req, activity_layer::loadDynaicMap::Response &resp)
{
    resp.success = false;
    try
    {
        std::ifstream ifs;
        ifs.open(req.path.c_str());
        std::vector<std::vector<double> > loadedObject;
        loadedObject.push_back(std::vector<double>());
        std::string line;
        int i = 0;
        while( std::getline(ifs,line))
        {
            std::istringstream iss(line);
            double val;
            while(iss >> val)
            {
                loadedObject[i].push_back(val);
            }

            i++;

            loadedObject.push_back(std::vector<double>());
        }
        _map_mutex.lock();
        _map->deserialize(loadedObject);
        _map_mutex.unlock();
        resp.success = true;
        ROS_INFO("LOAD DYNAMIC MAP - SUCCESS");
    }
    catch(...)
    {
        ROS_ERROR("LOAD DYNAMIC MAP - FAILED!");
    }

    return true;
}

}  // namespace layered_costmap_2d


