#include "pmac_learner.h"

Pmac_learner::Pmac_learner(int sizeX=2, int sizeY =2, double resolution=2)
    : grid(sizeX, sizeY, resolution)
{
    update_time = ros::Time::now().toNSec();
}

bool Pmac_learner::getCellValue(int x, int y, unsigned char& cellValueOutput)
{
    Pmac_cell* cell = grid.readCell(x,y);
    if (cell != NULL && cell->observationSum() >= minObsValue)
    {
        ros::Time currentTime = ros::Time::now();
        // Determine number of projection steps
        unsigned steps = (unsigned) (((currentTime.toSec() - cell->getLastObservation()) /((double)UPDATE_INTERVAL / 1e9) )+0.5);

        if(steps == 0)
            steps = 1;

        double occupancy_prob = 0;
        if(steps > 10 && steps > cell->getMixingTime())
        {
            occupancy_prob = cell->getLongTermOccupancyProb();
        }
        else
        {
            occupancy_prob = cell->getProjectedOccupancyProbability(steps);
        }



        if(occupancy_prob < 0) {
            occupancy_prob = 0;
        }
        else if(occupancy_prob > 1) {
            occupancy_prob = 1;
        }
        cellValueOutput = 252 * occupancy_prob;
        return true;
    }
    else
    {
        return false;
    }
}

void Pmac_learner::addObservationMap(Observation_interface* observation)
{
    //ROS_ERROR("time since update %i",(ros::Time::now().toNSec() - update_time));
    ros::Time t = ros::Time::now();
    if( (t.toNSec()  - update_time) > UPDATE_INTERVAL)
    {
        int max_x, max_y, min_y, min_x;
        observation->loadUpdateBounds( min_x,  max_x,  min_y,  max_y);
        if(max_x >= 0 && min_y >= 0 && max_y >= 0 && min_x >= 0)
        {
            for(int y = min_y; y <= max_y ; y++){
                for(int x = min_x; x <= max_x; x++){
                    double occupancy_prob = observation->getOccupancyPrabability(x,y);
                    if (occupancy_prob >= 0) {
                        Pmac_cell* cell = grid.editCell(x,y);
                        cell->addProbability(occupancy_prob,t.toSec());
                    }
                }
            }
        }
        observation->resetEditLimits();
    }
}

void Pmac_learner::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    grid.loadUpdateBounds(xMin, xMax, yMin, yMax);
}

double Pmac_learner::getOccupancyPrabability(int x, int y)
{
    Pmac_cell* cell = grid.readCell(x,y);
    if (cell != NULL)
    {
        //return cell->getLongTermOccupancyProb();
        double val = cell->getProjectedOccupancyProbability();
        return val;
    }
    else
        return -1;
}

void Pmac_learner::resetEditLimits()
{
    grid.resetUpdateBounds();
}

void Pmac_learner::initCell(int x, int y, Initial_values value)
{
    switch(value)
    {
        case Free:
            grid.editCell(x,y)->init(0,50);
            break;
        case Unknown:
            break;
        case Obstacle:
            grid.editCell(x,y)->init(200,0);
            break;
        default:
            break;
    }
}
