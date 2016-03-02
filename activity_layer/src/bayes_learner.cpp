#include "bayes_learner.h"

Bayes_learner::Bayes_learner()
{
}

Bayes_learner::Bayes_learner(int size_x, int size_y, double resolution)
    :_grid(size_x, size_y, resolution)
{
}

bool Bayes_learner::getCellValue(int x, int y, unsigned char & cellValueOutput)
{
    Probablistic_cell* cell = _grid.readCell(x,y);
    if(cell)
    {
        double prob = cell->getProbForOccupied(false);
        cellValueOutput = 255 * prob;
        return true;
    }
    return false;
}


void Bayes_learner::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
     _grid.loadUpdateBounds(xMin, xMax, yMin, yMax);
}

void Bayes_learner::addObservationMap(Observation_interface* observation)
{
    //ROS_ERROR("time since update %i",(ros::Time::now().toNSec() - update_time));
    ros::Time t = ros::Time::now();
    //if( (t.toNSec()  - update_time) > UPDATE_INTERVAL)
    {
        update_time = ros::Time::now().toNSec();
        int max_x, max_y, min_y, min_x;
        observation->loadUpdateBounds( min_x,  max_x,  min_y,  max_y);
        if(max_x >= 0 && min_y >= 0 && max_y >= 0 && min_x >= 0)
        {
            for(int y = min_y; y <= max_y ; y++){
                for(int x = min_x; x <= max_x; x++){
                    double occupancy_prob = observation->getOccupancyPrabability(x,y);
                    if (occupancy_prob >= 0) {
                        // calculate log odds
                        double log_odds = std::log(occupancy_prob / (1 - occupancy_prob));
                        Probablistic_cell* cell = _grid.editCell(x,y);
                       cell->addMeasurement(log_odds);
                    }
                }
            }
        }
        observation->resetEditLimits();
    }
}

void Bayes_learner::resetEditLimits()
{
    _grid.resetUpdateBounds();
}

void Bayes_learner::initCell(int x, int y, Initial_values value)
{
    /*
    switch(value)
    {
    case Free:
        _grid.editCell(x,y)->addMeasurement(_LOG_ODDS_FREE);
        break;
    case Unknown:
        _grid.editCell(x,y)->addMeasurement(0);
        break;
    case Obstacle:
        _grid.editCell(x,y)->addMeasurement(std::log(_INITIAL_OCCUPIED_PORBABILITY / (1- _INITIAL_OCCUPIED_PORBABILITY)));
        break;
    default:
        ROS_ERROR("UNKNOWN INITIAL VALUE - BAYES LEARNER");
        break;
    }
    */
    _grid.editCell(x,y)->addMeasurement(0.5);
}

void Bayes_learner::deserialize(const std::vector<std::vector<double> >& values)
{

}

std::vector<std::vector<double> > Bayes_learner::serialize()
{
    return std::vector<std::vector<double> >();
}
