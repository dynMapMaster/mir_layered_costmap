#include "probabilistic_filter.h"
#include <cmath>

Probabilistic_filter::Probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev)
{
    // -11.3401303868406
    _map = new Grid_structure<Probablistic_cell>(xDim,yDim,resolution);
    _laser_noise_var = laserStdDev * laserStdDev;
    _laser_noise_std_dev = laserStdDev;
    // Setup sensormodel lookup table
    // standard deviation = 0.025;

    _sensor_model.push_back(-6.56277460764592);
    _sensor_model.push_back(-1.67674182001805);
    _sensor_model.push_back(1.45884504871398);
    _sensor_model.push_back(4.71462867644434);
    _sensor_model.push_back(4.94875989037817);

    _sensor_model_occupancy_goal_index = 2;
}

Probabilistic_filter::~Probabilistic_filter()
{
    delete _map;
}

bool Probabilistic_filter::enforceBounds(int& x, int& y)
{
    bool changed = false;
    if(x < 0)
    {
        changed = true;
        x = 0;
    }
    else if(x >= _map->sizeX())
    {
        changed = true;
        x = _map->sizeX() - 1;
    }
    if(y < 0)
    {
        changed = true;
        y = 0;
    }
    else if(y >= _map->sizeY())
    {
        changed = true;
        y = _map->sizeY() - 1;
    }
    return changed;
}

void Probabilistic_filter::raytrace(int x0, int y0, int x1, int y1, bool markEnd)
{   
    bresenham2Dv0(x0,y0,x1,y1,markEnd);
}

double Probabilistic_filter::getOccupancyPrabability(int x, int y)
{
    Probablistic_cell* cell = _map->readCell(x,y);
    double result = -1;
    if(cell != NULL)
    {
        result = cell->getProbForOccupied();
        cell->resetCell();
    }
    return result;
}

void Probabilistic_filter::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    _map->loadUpdateBounds(xMin,xMax,yMin,yMax);
}

inline void Probabilistic_filter::bresenham2Dv0(int x1, int y1, int x2, int y2, bool markEnd)
{
    if(x1 == x2 && y1 == y2)
    {
        ROS_ERROR("RAYTRACE TO SAME CELL");
        return;
    }

    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) *2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;

    // Enforce bounds OBS ON X2 Y2
    bool out_of_bounds = enforceBounds(x2,y2);


    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        int pre_kernel_goal = x2 - (out_of_bounds || !markEnd ? 0 : _sensor_model.size()/2) * ix;

        while (x1 != pre_kernel_goal)
        {
            // Mark Position clear
            try // remove to improve performance
            {
                _map->editCell(x1,y1)->addMeasurement(_LOG_ODDS_FREE);
            }
            catch(const char* s)
            {
               ROS_ERROR("Filter marking goal error %s", s);
               return;
            }

            if ((error >= 0) && (error || (ix > 0)))
            {
                error -= delta_x;
                y1 += iy;
            }
            // else do nothing

            error += delta_y;
            x1 += ix;
        }


        if(!out_of_bounds && markEnd)
        {
            int sensor_model_end = pre_kernel_goal + _sensor_model.size();

            for(std::vector<double>::iterator sensor_ite = _sensor_model.begin(); sensor_ite != _sensor_model.end() && x1 != sensor_model_end ;sensor_ite++)
            {
                if(!(x1 >= 0 && y1 >= 0 && x1 < _map->sizeX() && y1 < _map->sizeY()))
                {
                    break;
                }

                // Mark Position occupied by sensor model
                _map->editCell(x1,y1)->addMeasurement(*sensor_ite);

                if ((error >= 0) && (error || (ix > 0)))
                {
                    error -= delta_x;
                    y1 += iy;
                }
                // else do nothing

                error += delta_y;
                x1 += ix;
            }
        }
        else
        {
            // Mark Position clear if no obstacle at target ie max range
            _map->editCell(x1,y1)->addMeasurement(_LOG_ODDS_FREE);
        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));


        int pre_kernel_goal = y2 - (out_of_bounds || !markEnd ? 0 : _sensor_model.size()/2) * iy;

        while (y1 != pre_kernel_goal)
        {

            // Mark Position clear
            try // remove to improve performance
            {
                _map->editCell(x1,y1)->addMeasurement(_LOG_ODDS_FREE);
            }
            catch(const char* s)
            {
               ROS_ERROR("Filter marking goal error %s", s);
               return;
            }

            if ((error >= 0) && (error || (iy > 0)))
            {
                error -= delta_y;
                x1 += ix;
            }
            // else do nothing

            error += delta_x;
            y1 += iy;
        }

        if(!out_of_bounds && markEnd)
        {
            int sensor_model_end = pre_kernel_goal + _sensor_model.size();

            for(std::vector<double>::iterator sensor_ite = _sensor_model.begin(); sensor_ite != _sensor_model.end() && y1 != sensor_model_end ;sensor_ite++)
            {
                if(!(x1 >= 0 && y1 >= 0 && x1 < _map->sizeX() && y1 < _map->sizeY()))
                {
                    break;
                }

                // Mark Position occupied by sensor model
                _map->editCell(x1,y1)->addMeasurement(*sensor_ite);

                if ((error >= 0) && (error || (iy > 0)))
                {
                    error -= delta_y;
                    x1 += ix;
                }
                // else do nothing

                error += delta_x;
                y1 += iy;
            }
        }
        else
        {
            // Mark Position clear if no obstacle at target ie max range
            _map->editCell(x1,y1)->addMeasurement(_LOG_ODDS_FREE);
        }
    }
}



void Probabilistic_filter::resetEditLimits()
{
    _map->resetUpdateBounds();
}
