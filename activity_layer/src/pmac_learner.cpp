#include "pmac_learner.h"

Pmac_learner::Pmac_learner(int sizeX=2, int sizeY =2, double resolution=2)
    : grid(sizeX, sizeY, resolution), sse_score(0.0), scored_observations(0)
{
    update_time = ros::Time::now().toNSec();
}

bool Pmac_learner::getCellValue(int x, int y, unsigned char& cellValueOutput)
{
    Pmac_cell* cell = grid.readCell(x,y);
    if (cell != NULL && cell->observationSum() >= MIN_OBS_VALUE)
    {
        ros::Time currentTime = ros::Time::now();
        // Determine number of projection steps
        unsigned steps = (unsigned) (((currentTime.toSec() - cell->getLastObservationTime()) /((double)UPDATE_INTERVAL / 1e9) )+0.5);

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
        cellValueOutput = 255 * occupancy_prob;

        translateOcc(cellValueOutput);

        return true;
    }
    else
    {
        return false;
    }
}

double Pmac_learner::getPredictScore()
{
    if(scored_observations > 0)
        return sse_score / ((double)scored_observations);
    else
        return 0.0;
}

void Pmac_learner::addObservationMap(Observation_interface* observation)
{
    //ROS_ERROR("time since update %i",(ros::Time::now().toNSec() - update_time));
    ros::Time t = ros::Time::now();
    if( (t.toNSec()  - update_time) > UPDATE_INTERVAL)
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
                        double predicted_occupancy_prob = getOccupancyPrabability(x,y);
                        Pmac_cell* cell = grid.editCell(x,y);
                        if(predicted_occupancy_prob > 0.5 || occupancy_prob > 0.5){
                            sse_score += std::pow(occupancy_prob - predicted_occupancy_prob,2);
                            scored_observations++;
                        }
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
        uint64_t prev_reading_time = cell->getLastObservationTime() * 1e9;
        uint64_t time_now = ros::Time::now().toNSec();
        uint64_t time_span = time_now - prev_reading_time;
        unsigned steps = time_span / UPDATE_INTERVAL + 0.5;
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
        return occupancy_prob;
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
            grid.editCell(x,y)->init(50,0);
            break;
        default:
            break;
    }
}

void Pmac_learner::translateOcc(unsigned char& value)
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

std::vector<std::vector<double> > Pmac_learner::serialize()
{
    std::vector<std::vector<double> > result;
    result.reserve(grid.sizeX()*grid.sizeY()+1);
    // Store size of grid and resolution
    std::vector<double> size;
    size.push_back(grid.sizeX());
    size.push_back(grid.sizeY());
    size.push_back(grid.resolution());
    result.push_back(size);

    // Store grid values
    for(int y = 0; y < grid.sizeY();y++){
        for(int x = 0; x < grid.sizeX();x++){
            Pmac_cell* cell = grid.readCell(x,y);
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

void Pmac_learner::deserialize(const std::vector<std::vector<double> >& values)
{
    if(values.size() < 1 || values[0].size() < 3)
    {
        ROS_ERROR("Pmac_learner - FIRST LINE IN WRONG FORMAT!!");
        return;
    }
    // create gridstructure in correct size
    //grid = Grid_structure<Pmac_cell>(values[0][0],values[0][1],values[0][2]);

    // Make sure grid sizes match
    if(grid.sizeX() != (int)values[0][0] || grid.sizeY() != (int)values[0][1] || std::fabs(grid.resolution()-values[0][2]) > 0.01)
    {
        ROS_ERROR("LOADED MAP AND ORIGINAL DOES NOT MATCH IN SIZE OR RESOLUTION %f %f %f ",values[0][0], values[0][1],  values[0][2]);
        ROS_ERROR("CURRENT %i %i %f ",grid.sizeX(), grid.sizeY(), grid.resolution());
        return;
    }

    for(int i = 1; i < values.size();i++){
        if(values[i].size() == 5)
        {

            // calculate x-y Value
            int y = (i-1) / grid.sizeX();
            int x = (i-1) % grid.sizeX();
            Pmac_cell* cell = grid.editCell(x,y);
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
