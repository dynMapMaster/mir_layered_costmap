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

#include <fstream>;
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <algorithm>

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
    poseSubscriber = g_nh.subscribe(poseTopicName, 50, &ActivityLayer::poseCB,this);
    poseArraySubscriber = g_nh.subscribe("/particlecloud",1,&ActivityLayer::poseArrayCB,this);
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
    matchSize();
}

void ActivityLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{

}

ActivityLayer::~ActivityLayer()
{

    if (dsrv_)
        delete dsrv_;

    if(_observation_map)
        delete _observation_map;

    if(_map)
        delete _map;

}
void ActivityLayer::reconfigureCB(layered_costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{

}
double prev_recive_time;
void ActivityLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{    
    //if(poseIsAccurate)
    {
        sensor_msgs::LaserScan message = *raw_message;
        ros::Time recive_time = message.header.stamp;
        double now = recive_time.toSec();
        if( (now - prev_recive_time) > 0.11)
        {
            ROS_WARN("Time between scans: %f",now - prev_recive_time);
        }
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
        buffer->lock();
        buffer->bufferCloud(cloud);
        buffer->unlock();
        _sensor_frame_id = message.header.frame_id;
        laserScanWaitingCounter++;
        if(laserScanWaitingCounter > 0)
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
                    raytrace(buffer[o]);
                }
            }
            laserScanWaitingCounter = 0;
        }
    }
    /*
    else {
        ROS_WARN("Skipping update of dynamic map since robot pose is too inaccurate");
    }
    */
    _map->addObservationMap(_observation_map);
}

void ActivityLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
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
    //_map->addObservationMap(_observation_map);

    //ROS_INFO("Updating bound from activity layer");
    int layerMinX, layerMaxX, layerMinY, layerMaxY;
    _map->loadUpdateBounds(layerMinX, layerMaxX, layerMinY, layerMaxY);
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
    //ROS_INFO("Updating cost from activity layer %i, %i, %i, %i",min_i, min_j, max_i, max_j);
    unsigned char* master_array = master_grid.getCharMap();
    unsigned char val = 0;
    unsigned int span = master_grid.getSizeInCellsX();
    for (int j = min_j; j < max_j; j++)
    {
        //ROS_INFO("j=%i",j);
        unsigned int it = j * span + min_i;
        for (int i = min_i; i < max_i; i++)
        {
            if (_map->getCellValue(i,j,val)){
                unsigned char old_cost = master_array[it];
                //if (old_cost == NO_INFORMATION || old_cost < val)
                master_array[it] =  val;
            }
            else
            {
                master_array[it] = 96;
            }
            it++;
        }
    }

    _map->resetEditLimits();
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

int update_count = 0;
void ActivityLayer::raytrace(const Observation& observation)
{

    //if(_pose_array.poses.size() > 0 && update_count < 1)
    {
        Costmap2D* master = layered_costmap_->getCostmap();
#if SENSOR_MODEL_TYPE == POSE_ARRAY
        ros::Time raytrace_time = ros::Time::now();
        if((raytrace_time.toSec() - cloud_recive_time.toSec()) > 0.3)
        {
            ROS_WARN("Skipping since time since last pose array recived is too big: %f",(raytrace_time.toSec() - cloud_recive_time.toSec()));
            return;
        }
        // get robot base link transform
        tf::StampedTransform tf_map_to_robot;
        tf_->waitForTransform(_global_frame,ros::Time::now(),"base_link",ros::Time::now(),_global_frame,ros::Duration(5.0));
        try
        {
            tf_->lookupTransform(_global_frame,ros::Time::now(),"base_link",ros::Time::now(),_global_frame,tf_map_to_robot);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("TF returned a transform exception between frame %s and %s: %s",
                     _global_frame.c_str(), "base_link", ex.what());
            return;
        }
        update_count++;
        tf::Transform tf_robot_to_map = tf_map_to_robot.inverse();
        // create a measurement cloud position wrt. the robot's base
        pcl::PointCloud<pcl::PointXYZ>::Ptr mesurement_cloud_in_robot_frame(new pcl::PointCloud<pcl::PointXYZ>());
        pcl_ros::transformPointCloud(*observation.cloud_, *mesurement_cloud_in_robot_frame, tf_robot_to_map);
        // trace out each scan for each particle pose
        for (int j = 0; j < _pose_array.poses.size(); ++j) {
            //ROS_INFO("Particle no. %i",j);
            geometry_msgs::PoseStamped particle_pose;
            particle_pose.header = observation.cloud_->header;
            particle_pose.pose = _pose_array.poses[j];
            particle_pose.pose.position.z = 0;
            tf::Transform tf_map_to_particle;
            tf_map_to_particle.setOrigin(tf::Vector3(particle_pose.pose.position.x, particle_pose.pose.position.y, particle_pose.pose.position.z));
            tf_map_to_particle.setRotation(tf::Quaternion( particle_pose.pose.orientation.x, particle_pose.pose.orientation.y, particle_pose.pose.orientation.z, particle_pose.pose.orientation.w));
            // get transform
            tf::Transform tf_robot_to_particle = tf_robot_to_map * tf_map_to_particle;
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_ros::transformPointCloud(*mesurement_cloud_in_robot_frame, *particle_cloud_tmp, tf_robot_to_particle);
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_ros::transformPointCloud(*particle_cloud_tmp, *particle_cloud, tf_map_to_robot);
            _observation_map->_raytrace_weight = _pose_array.poses[j].position.z;
            for(size_t i = 0; i < particle_cloud->size();i++){
                //calculate range
                double range = sqrt(pow(observation.origin_.x - observation.cloud_->points[i].x,2)+pow(observation.origin_.y - observation.cloud_->points[i].y,2));

                bool mark_end = (fabs(range-_max_range) < 0.001 ? false : true);
                int x0,y0, x1, y1;
                master->worldToMapEnforceBounds(observation.origin_.x,observation.origin_.y,x0,y0);
                master->worldToMapEnforceBounds(particle_cloud->points[i].x,particle_cloud->points[i].y,x1,y1);
                try
                {
                    _observation_map->_angle_std_dev = _angle_std_dev;
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
        }
#else
        for(size_t i = 0; i < observation.cloud_->size();i++){
            //calculate range
            double range = sqrt(pow(observation.origin_.x - observation.cloud_->points[i].x,2)+pow(observation.origin_.y - observation.cloud_->points[i].y,2));
            bool mark_end = (fabs(range-_max_range) < 0.001 ? false : true);

            int x0,y0, x1, y1;
            master->worldToMapEnforceBounds(observation.origin_.x,observation.origin_.y,x0,y0);
            master->worldToMapEnforceBounds(observation.cloud_->points[i].x,observation.cloud_->points[i].y,x1,y1);
            try
            {
                _observation_map->_angle_std_dev = _angle_std_dev;
                //if(_x_std_dev < 0 || _y_std_dev < 0) // debug
                //    return;
                _observation_map->_x_std_dev = 9;//_x_std_dev;
                _observation_map->_y_std_dev = 9;//_y_std_dev;
                _observation_map->coneRayTracePoseUncertain(observation.origin_.x, observation.origin_.y,
                                                            observation.cloud_->points[i].x, observation.cloud_->points[i].y, 1*_angle_std_dev, mark_end);
                return;
#if SENSOR_MODEL_TYPE == LINE_MODEL
                _observation_map->raytrace(x0,y0,x1,y1,mark_end);
#elif SENSOR_MODEL_TYPE == KERNEL_MODEL
                _observation_map->raytrace(x0,y0,x1,y1,false);
                if(mark_end)
                    _observation_map->coneRayTrace(observation.origin_.x, observation.origin_.y,
                                                   observation.cloud_->points[i].x, observation.cloud_->points[i].y, 1*_angle_std_dev, mark_end);
#elif SENSOR_MODEL_TYPE == CONE_MODEL
                _observation_map->coneRayTrace(observation.origin_.x, observation.origin_.y,
                                               observation.cloud_->points[i].x, observation.cloud_->points[i].y, 1*_angle_std_dev, mark_end);
#endif
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
#endif
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
    Costmap2D* master = layered_costmap_->getCostmap();
    nav_msgs::OccupancyGrid grid = requestMap();
    master->resizeMap(grid.info.width, grid.info.height, grid.info.resolution,
                      grid.info.origin.position.x, grid.info.origin.position.y);
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
    // request map from mapserver

    // Initialize _map
    if(prev_x_size >= grid.info.width && prev_y_size >= grid.info.height)
    {
        for(int x = 0; x < grid.info.width; x++){
            for(int y = 0; y < grid.info.height; y++){
                Costmap_interpretator::Initial_values val = determineInitialValue(grid.data[y * grid.info.width + x]);
                u_char learned_value;
                if(_map->getCellValue(x,y,learned_value) == false )
                    _map->initCell(x,y,val);
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

// sort using a custom function object
struct weightMore{
    inline bool operator()(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
    {
        return p1.position.z > p2.position.z;
    }
} ;

void ActivityLayer::poseArrayCB(geometry_msgs::PoseArray pose)
{
    std::sort(pose.poses.begin(), pose.poses.end(), weightMore());

    int used_weights = 30;

    if(used_weights > pose.poses.size())
        used_weights = pose.poses.size();
    /*

    // use particles which describes part of probability
    double weight_sum = 0;
    const double described_probability = 0.191;//0.3413;//0.682;
    int i;
    for (i = 0; weight_sum < described_probability; ++i) {
        weight_sum += pose.poses[i].position.z;
    }
    used_weights = i + 1;

    pose.poses.resize(used_weights);
    //ROS_INFO("uses %i particles",used_weights);

    double excluded_weight_addition = (1-weight_sum) / used_weights;
    //compensate for excluded particles
    for (int i = 0; i < pose.poses.size(); ++i) {
        pose.poses[i].position.z += excluded_weight_addition;
    }

    // normalize weights
    /*
    double total_weight = 0;
    for (int i = 0; i < pose.poses.size(); ++i) {
        total_weight += pose.poses[i].position.z;
    }
    for (int i = 0; i < pose.poses.size(); ++i) {
        pose.poses[i].position.z /= total_weight;
    }
    */
    _pose_array = pose;
    cloud_recive_time = ros::Time::now();
}

bool ActivityLayer::saveDynamicMap(activity_layer::saveDynaicMap::Request &req, activity_layer::saveDynaicMap::Response &resp)
{
    resp.success = false;
    try
    {
        std::vector<std::vector<double> > mapSerialized = _map->serialize();
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
        _map->deserialize(loadedObject);
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


