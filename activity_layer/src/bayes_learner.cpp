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
    _grid.editCell(x,y)->addMeasurement(0.0);
}

void Bayes_learner::deserialize(const std::vector<std::vector<double> >& values)
{
    if(values.size() < 1 || values[0].size() < 3)
    {
        ROS_ERROR("Pmac_learner - FIRST LINE IN WRONG FORMAT!!");
        return;
    }
    // create gridstructure in correct size

    // Make sure grid sizes match
    if(_grid.sizeX() != (int)values[0][0] || _grid.sizeY() != (int)values[0][1] || std::fabs(_grid.resolution()-values[0][2]) > 0.01)
    {
        ROS_ERROR("LOADED MAP AND ORIGINAL DOES NOT MATCH IN SIZE OR RESOLUTION %f %f %f ",values[0][0], values[0][1],  values[0][2]);
        ROS_ERROR("CURRENT %i %i %f ",_grid.sizeX(), _grid.sizeY(), _grid.resolution());
        return;
    }

    for(int i = 1; i < values.size();i++){
        if(values[i].size() == 1)
        {
            // calculate x-y Value
            int y = (i-1) / _grid.sizeX();
            int x = (i-1) % _grid.sizeX();
            Probablistic_cell* cell = _grid.editCell(x,y);
            if(cell)
                cell->deserialize(values[i]);
            else
                ROS_ERROR("CELL WAS NULL!");
        }
        else if(values[i].size() == 1)
        {

        }
        else
        {
            ROS_ERROR("LOADED CELL WRONG SIZE %i", values[i].size());
        }

    }
}

std::vector<std::vector<double> > Bayes_learner::serialize()
{
    std::vector<std::vector<double> > result;
    result.reserve(_grid.sizeX()*_grid.sizeY()+1);
    // Store size of grid and resolution
    std::vector<double> size;
    size.push_back(_grid.sizeX());
    size.push_back(_grid.sizeY());
    size.push_back(_grid.resolution());
    result.push_back(size);

    // Store grid values
    for(int y = 0; y < _grid.sizeY();y++){
        for(int x = 0; x < _grid.sizeX();x++){
            Probablistic_cell* cell = _grid.readCell(x,y);
            std::vector<double> cellVal;
            if(cell)
            {
                cellVal = cell->serialize();
            }
            else
            {
                // OBS IF SINGLE VECTOR SERIALIZATION - CONSIDER
                cellVal.push_back(0);
            }
            result.push_back(cellVal);
        }
    }
    return result;
}
