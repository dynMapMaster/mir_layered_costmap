#include "probabilistic_filter.h"
#include <cmath>
#include <angles/angles.h>
#include <cfloat>
Probabilistic_filter::Probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev, double origin_x, double origin_y)
{
    _map = new Grid_structure<Probablistic_cell>(xDim,yDim,resolution, origin_x, origin_y);
    _laser_noise_var = laserStdDev * laserStdDev;
    _laser_noise_std_dev = laserStdDev;

    // Setup sensormodel lookup table
    _sensor_model.push_back(0.4055);
    //_sensor_model.push_back(2.1972);
    _sensor_model_occupancy_goal_index = 0;
    _angle_std_dev =  15 * M_PI/180.0;
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

inline double Probabilistic_filter::getRangeWeight(int x1, int y1, int ori_x, int ori_y)
{
    const double dx = x1 - ori_x;
    const double dy = y1 - ori_y;
    const double dist = _map->resolution()*sqrt(dx*dx+dy*dy);

    return (1-std::min(2*_angle_std_dev*dist+2*(_x_std_dev+_y_std_dev),1.0));

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

    int ori_x = x1, ori_y = y1;

    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        int pre_kernel_goal = x2 - (out_of_bounds || !markEnd ? 0 : _sensor_model.size()/2) * ix;

        while (x1 != pre_kernel_goal)
        {
            double weight = getRangeWeight(x1,y1,ori_x,ori_y);
            // Mark Position clear
            try // remove to improve performance
            {
                double value = weight*_LOG_ODDS_FREE;
                if(std::fabs(value) <= _RAYTRACE_INSERT_TRESHOLD)
                    return;

                _map->editCell(x1,y1)->addMeasurement(value);
            }
            catch(const char* s)
            {
                ROS_ERROR("Filter marking goal error 1 %s", s);
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
        double weight = getRangeWeight(x1,y1,ori_x,ori_y);
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
                try // remove to improve performance
                {
                    double value = weight*(*sensor_ite);
                    if(std::fabs(value) <= _RAYTRACE_INSERT_TRESHOLD)
                        return;

                    _map->editCell(x1,y1)->addMeasurement(value);
                }
                catch(const char* s)
                {
                    ROS_ERROR("Filter marking goal error 2 %s", s);
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
        }
        else
        {
            try // remove to improve performance
            {
                // Mark Position clear if no obstacle at target ie max range
                _map->editCell(x1,y1)->addMeasurement(weight*_LOG_ODDS_FREE);
            }
            catch(const char* s)
            {
                ROS_ERROR("Filter marking goal error 3 %s", s);
                return;
            }
        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));

        int pre_kernel_goal = y2 - (out_of_bounds || !markEnd ? 0 : _sensor_model.size()/2) * iy;

        while (y1 != pre_kernel_goal)
        {
            double weight = getRangeWeight(x1,y1,ori_x,ori_y);
            // Mark Position clear
            try // remove to improve performance
            {
                double value = weight*_LOG_ODDS_FREE;
                if(std::fabs(value) <= _RAYTRACE_INSERT_TRESHOLD)
                    return;

                _map->editCell(x1,y1)->addMeasurement(value);
            }
            catch(const char* s)
            {
                ROS_ERROR("Filter marking goal error 4 %s", s);
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
        double weight = getRangeWeight(x1,y1,ori_x,ori_y);
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
                try // remove to improve performance
                {
                    double value = weight*(*sensor_ite);
                    if(std::fabs(value) <= _RAYTRACE_INSERT_TRESHOLD)
                        return;

                    _map->editCell(x1,y1)->addMeasurement(value);
                }
                catch(const char* s)
                {
                    ROS_ERROR("Filter marking goal error 5 %s", s);
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
        }
        else
        {
            try // remove to improve performance
            {
                // Mark Position clear if no obstacle at target ie max range
                _map->editCell(x1,y1)->addMeasurement(weight*_LOG_ODDS_FREE);
            }
            catch(const char* s)
            {
                ROS_ERROR("Filter marking goal error 6 %s", s);
                return;
            }
        }
    }
}



void Probabilistic_filter::resetEditLimits()
{
    _map->resetUpdateBounds();
}
