/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
#include <user_priority_layer.h>
#include <layered_costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(user_priority_map, UserPriorityLayer, user_priority_map::UserPriorityLayer,layered_costmap_2d::Layer)

namespace user_priority_map
{

    UserPriorityLayer::UserPriorityLayer() : dsrv_(NULL)  {} //

    UserPriorityLayer::~UserPriorityLayer()
    {
        if (dsrv_)
            delete dsrv_;
    }

    void UserPriorityLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;
        current_ = true;

        global_frame_ = layered_costmap_->getGlobalFrameID();

        std::string map_topic;
        nh.param("map_topic", map_topic, std::string("/map"));
        nh.param("first_map_only", first_map_only_, false);
        nh.param("subscribe_to_updates", subscribe_to_updates_, false);

        // we'll subscribe to the latched topic that the map server uses
        ROS_INFO("Requesting the map...");
        map_sub_ = g_nh.subscribe(map_topic, 1, &UserPriorityLayer::incomingMap, this);
        map_received_ = false;
        has_updated_data_ = false;

        ros::Rate r(10);
        while (!map_received_ && g_nh.ok())
        {
            ros::spinOnce();
            r.sleep();
        }

        ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

        if (subscribe_to_updates_)
        {
            ROS_INFO("Subscribing to updates");
            map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &UserPriorityLayer::incomingUpdate, this);
        }

        if (dsrv_)
        {
            delete dsrv_;
        }

        dsrv_ = new dynamic_reconfigure::Server<layered_costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<layered_costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &UserPriorityLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void UserPriorityLayer::reconfigureCB(layered_costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        if (config.enabled != enabled_)
        {
            enabled_ = config.enabled;
            has_updated_data_ = true;
            x_ = y_ = 0;
            width_ = size_x_;
            height_ = size_y_;
        }
    }

    void UserPriorityLayer::matchSize()
    {
        // If we are using rolling costmap, the static map size is
        //   unrelated to the size of the layered costmap
        if (!layered_costmap_->isRolling())
        {
            Costmap2D* master = layered_costmap_->getCostmap();
            resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                      master->getOriginX(), master->getOriginY());
        }
    }

    unsigned char UserPriorityLayer::interpretValue(unsigned char value)
    {
        // check if the static value is above the unknown or lethal thresholds
        unsigned char interpreted_value;
        switch(value)
        {
        case PREFERRED:
        case UNPREFERRED:
        case CRITICAL:
        case FORBIDDEN:
            interpreted_value = value;
            break;
        default:
            interpreted_value = NO_INFORMATION;
            break;
        }
        //ROS_INFO("Value = %d",interpreted_value);
        return interpreted_value;
    }

    void UserPriorityLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
    {
        unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

        ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

        // resize costmap if size, resolution or origin do not match
        Costmap2D* master = layered_costmap_->getCostmap();
        if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
                                               master->getSizeInCellsY() != size_y ||
                                               master->getResolution() != new_map->info.resolution ||
                                               master->getOriginX() != new_map->info.origin.position.x ||
                                               master->getOriginY() != new_map->info.origin.position.y ||
                                               !layered_costmap_->isSizeLocked()))
        {
            // Update the size of the layered costmap (and all layers, including this one)
            ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
            layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                        new_map->info.origin.position.y, true);
        }
        else if (size_x_ != size_x || size_y_ != size_y ||
                 resolution_ != new_map->info.resolution ||
                 origin_x_ != new_map->info.origin.position.x ||
                 origin_y_ != new_map->info.origin.position.y)
        {
            // only update the size of the costmap stored locally in this layer
            ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
            resizeMap(size_x, size_y, new_map->info.resolution,
                      new_map->info.origin.position.x, new_map->info.origin.position.y);
        }

        unsigned int index = 0;

        // initialize the costmap with static data
        for (unsigned int i = 0; i < size_y; ++i)
        {
            for (unsigned int j = 0; j < size_x; ++j)
            {
                unsigned char value = new_map->data[index];
                costmap_[index] = interpretValue(value);
                ++index;
            }
        }
        map_frame_ = new_map->header.frame_id;

        // we have a new map, update full size of map
        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        map_received_ = true;
        has_updated_data_ = true;

        // shutdown the map subscrber if firt_map_only_ flag is on
        if (first_map_only_)
        {
            ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
            map_sub_.shutdown();
        }
    }

    void UserPriorityLayer::incomingUpdate(const mirMsgs::OccupancyGridUpdateConstPtr& update)
    {
        unsigned int di = 0;
        for (unsigned int y = 0; y < update->height ; y++)
        {
            unsigned int index_base = (update->y + y) * size_x_;
            for (unsigned int x = 0; x < update->width ; x++)
            {
                unsigned int index = index_base + x + update->x;
                costmap_[index] = (update->data[di++]);
            }
        }
        x_ = update->x;
        y_ = update->y;
        width_ = update->width;
        height_ = update->height;
        has_updated_data_ = true;
    }

    void UserPriorityLayer::activate()
    {
        onInitialize();
    }

    void UserPriorityLayer::deactivate()
    {
        map_sub_.shutdown();
        if (subscribe_to_updates_)
            map_update_sub_.shutdown();
    }

    void UserPriorityLayer::reset()
    {
        if (first_map_only_)
        {
            has_updated_data_ = true;
        }
        else
        {
            deactivate();
            activate();
        }
    }

    void UserPriorityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                         double* max_x, double* max_y)
    {
        if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
            return;

        useExtraBounds(min_x, min_y, max_x, max_y);

        double wx, wy;

        mapToWorld(x_, y_, wx, wy);
        *min_x = std::min(wx, *min_x);
        *min_y = std::min(wy, *min_y);

        mapToWorld(x_ + width_, y_ + height_, wx, wy);
        *max_x = std::max(wx, *max_x);
        *max_y = std::max(wy, *max_y);

        has_updated_data_ = false;
    }

    void UserPriorityLayer::updateWithOverwrite(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;
        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                if (costmap_[it] != NO_INFORMATION)
                    master[it] = costmap_[it];
                it++;
            }
        }
    }

    void UserPriorityLayer::updateWithMax(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
      if (!enabled_)
        return;

      unsigned char* master_array = master_grid.getCharMap();
      unsigned int span = master_grid.getSizeInCellsX();

      for (int j = min_j; j < max_j; j++)
      {
        unsigned int it = j * span + min_i;
        for (int i = min_i; i < max_i; i++)
        {
          if (costmap_[it] == NO_INFORMATION){
            it++;
            continue;
          }

          unsigned char old_cost = master_array[it];
          if (old_cost == NO_INFORMATION || old_cost < costmap_[it])
            master_array[it] = costmap_[it];
          it++;
        }
      }
    }

    void UserPriorityLayer::updateCosts(layered_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!map_received_)
            return;

        if (!layered_costmap_->isRolling())
        {
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
            //updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        }
        else
        {
            // If rolling window, the master_grid is unlikely to have same coordinates as this layer
            unsigned int mx, my;
            double wx, wy;
            // Might even be in a different frame
            tf::StampedTransform transform;
            try
            {
                tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }
            // Copy map data given proper transformations
            unsigned int span = master_grid.getSizeInCellsX();
            for (unsigned int j = min_j; j < max_j; ++j)
            {
                unsigned int it = span*j+min_i;
                for (unsigned int i = min_i; i < max_i; ++i)
                {
                    // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
                    layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
                    // Transform from global_frame_ to map_frame_
                    tf::Point p(wx, wy, 0);
                    p = transform(p);
                    // Set master_grid with cell from map
                    if (worldToMap(p.x(), p.y(), mx, my))
                    {
                        if (costmap_[it] != NO_INFORMATION)
                            master_grid.setCost(i, j, getCost(mx, my));
                    }
                }
            }
        }
    }

}  // namespace user_priority_map

