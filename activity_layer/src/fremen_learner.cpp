#include "fremen_learner.h"
#include <std_msgs/Float32.h>
using namespace std;
Fremen_learner::Fremen_learner(int sizeX=2, int sizeY =2, double resolution=2)
    : grid(sizeX, sizeY, resolution), scored_observations(0)
{
    update_time = ros::Time::now().toNSec();
    ros::NodeHandle nh;
    currentErrorPub = nh.advertise<std_msgs::Float32>("current_map_error",10);
}

bool Fremen_learner::getCellValue(int x, int y, unsigned char & cellValueOutput)
{
    double occ_value = getOccupancyPrabability(x,y);
    if(occ_value >= 0)
    {
        cellValueOutput = 255 * occ_value;

        translateOcc(cellValueOutput);

        return true;
    }
    else
    {
        return false;
    }
}

void Fremen_learner::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{

}

void Fremen_learner::addObservationMap(Observation_interface* observation)
{
    //ROS_ERROR("time since update %i",(ros::Time::now().toNSec() - update_time));
    ros::Time t = ros::Time::now();
    if( (t.toNSec()  - update_time) > UPDATE_INTERVAL)
    {
        double current_sse, current_obs_number;
        update_time = ros::Time::now().toNSec();
        int max_x, max_y, min_y, min_x;
        observation->loadUpdateBounds( min_x,  max_x,  min_y,  max_y);
        if(max_x >= 0 && min_y >= 0 && max_y >= 0 && min_x >= 0)
        {
            for(int y = min_y; y <= max_y ; y++){
                for(int x = min_x; x <= max_x; x++){
                    double occupancy_prob = observation->getOccupancyPrabability(x,y);
                    if (occupancy_prob >= 0) {
                        double predicted_occupancy_prob = getOccupancyPrabability(x,y);
                        Fremen_cell* cell = grid.editCell(x,y);
                        if((predicted_occupancy_prob > 0.5 || occupancy_prob > 0.5) && predicted_occupancy_prob >= 0){
                            sse_score += std::pow(occupancy_prob - predicted_occupancy_prob,2);
                            scored_observations++;
                            current_sse += std::pow(occupancy_prob - predicted_occupancy_prob,2);
                            current_obs_number++;
                        }
                        cell->addProbability(occupancy_prob,t.toSec());
                    }
                }
            }
        }
        if(current_obs_number > 0 )
        {
            std_msgs::Float32 msg;
            msg.data = (float)(current_sse / current_obs_number);
            currentErrorPub.publish(msg);
        }
        observation->resetEditLimits();
    }
}

void Fremen_learner::resetEditLimits()
{
    grid.resetUpdateBounds();
}

double Fremen_learner::getPredictScore()
{
    if(scored_observations > 0)
        return sse_score / ((double)scored_observations);
    else
        return 0.0;
}

void Fremen_learner::initCell(int x, int y, Initial_values value)
{

}

void Fremen_learner::deserialize(const std::vector<std::vector<double> >& values)
{

}

vector<vector<double> > Fremen_learner::serialize()
{
    vector<vector<double> > result;
    vector<double> init_vals;
    init_vals.push_back(grid.sizeX());
    init_vals.push_back(grid.sizeY());
    init_vals.push_back(grid.resolution());
    result.push_back(init_vals);
    for(int y = 0; y < grid.sizeX();y++){
        for(int x = 0 ; x < grid.sizeX(); x++){
            vector<double> next_vector;
            Fremen_cell* cell = grid.readCell(x,y);
            if(cell != NULL)
            {
                result.push_back(cell->serialize());
            }
            else
            {
                next_vector.push_back(0);
                result.push_back(next_vector);
            }
        }
    }
    return result;
}

void Fremen_learner::translateOcc(unsigned char& value)
{
    if(value == 95) // Avoid making critical regions
    {
        value = 96;
    }
    else if(value >= OBSTACLE_THRESHOLD)
    {
        value = 255;
    }
}

double Fremen_learner::getOccupancyPrabability(int x, int y)
{
    Fremen_cell* cell = grid.readCell(x,y);
    if (cell != NULL)
    {
        double now = ros::Time::now().toSec();
        return cell->estimate(now);
    }
    else
        return -1.0;
}
