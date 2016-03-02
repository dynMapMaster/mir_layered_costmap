#include "probabilistic_filter.h"
#include <cmath>
#include <angles/angles.h>
Probabilistic_filter::Probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev, double origin_x, double origin_y)
{
    _map = new Grid_structure<Probablistic_cell>(xDim,yDim,resolution, origin_x, origin_y);
    _laser_noise_var = laserStdDev * laserStdDev;
    _laser_noise_std_dev = laserStdDev;
    // Setup sensormodel lookup table
    ROS_INFO("LOG odds free=%f",_LOG_ODDS_FREE);

#if SENSOR_MODEL_TYPE == LINE_MODEL
    #if USE_IDEAL_LINE_SENSOR_MODEL > 0
    double free_log = _LOG_ODDS_FREE;
    _sensor_model.push_back(free_log);
    _sensor_model.push_back(free_log);
    _sensor_model.push_back(0.4055);
    _sensor_model_occupancy_goal_index = 2;
    #else
    _sensor_model.push_back(-1.09818644066980);
    _sensor_model.push_back(-0.811877413304292);
    _sensor_model.push_back(0.724863718358982);
    _sensor_model.push_back(0.206581098035374);
    _sensor_model.push_back(0.000318349321477128);
    _sensor_model_occupancy_goal_index = 2;
    #endif
#endif

    _max_angle = 5 * M_PI/180.0;
    _angle_std_dev = _max_angle;
    ROS_INFO("max angle: %f", _max_angle);
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

double Probabilistic_filter::gamma(double theta)
{
    if(std::fabs(theta)>_max_angle)
        return 0.0;
    else
        return 1 - pow(theta/_max_angle, 2);
}

double Probabilistic_filter::delta(double phi)
{
    return 1;//1 - (1+tanh(2*(phi-_phi_v)))/2;
}

double Probabilistic_filter::gaussian_sensor_model(double r, double phi, double theta)
{
    double sigma_r = 0.025;
    double span = 0.3, min_prob = (1-span)/2;    
#if USE_RANGE_AND_NOISE_DECAY
    double free_weight = 1 - std::min(2*_max_angle*phi,1.0);
#else
    double free_weight = 1;
#endif
    double lbda = gamma(theta)*free_weight;
    double prob;
    if(phi < r - 2 * sigma_r)
        prob = min_prob + (1 - lbda) * (0.5)*span;
    else if(phi < r - sigma_r)
        prob = span * (lbda * 0.5 * std::pow((phi - (r - 2*sigma_r))/(sigma_r), 2)+(1-lbda)*.5) + min_prob;
    else if(phi < r + sigma_r) {
        double J = (r-phi)/(sigma_r);
        prob = span * (lbda * ((1 - (0.5)*std::pow(J,2)) - 0.5) + 0.5) + min_prob;
    }
    else
        prob = 0.5;
    return prob;
}

double Probabilistic_filter::kernel_sensor_model(double r, double phi, double theta)
{
    double sigma_r = 0.025;
    double span = 0.3, min_prob = (1-span)/2;
#if USE_RANGE_AND_NOISE_DECAY
    double free_weight = 1 - std::min(2*_max_angle*phi,1.0);
#else
    double free_weight = 1;
#endif
    double lbda = gamma(theta)*free_weight;
    double prob;
    if(phi < r -  sigma_r)
        //prob = min_prob + (1 - lbda) * (0.5)*span;
        prob = 0.5;
    else if(phi < r + sigma_r) {
        double J = (r-phi)/(sigma_r);
        prob = lbda * ((1 - (0.5)*std::pow(J,2)) - 0.5) + 0.5;
    }
    else
        prob = 0.5;
    return prob;
}

double Probabilistic_filter::sensor_model(double r, double phi, double theta)
{
    double lbda = delta(phi)*gamma(theta);
    double delta = 0.01;//_map->resolution();
    if(phi < r - 2 * delta * r)
        return (1 - lbda) * (0.5);
    else if(phi < r - delta * r)
        return lbda * 0.5 * std::pow((phi - (r - 2*delta*r))/(delta*r), 2)+(1-lbda)*.5;
    else if(phi < r + delta * r){
        double J = (r-phi)/(delta*r);
        return lbda * ((1 - (0.5)*std::pow(J,2)) - 0.5) + 0.5;
    }
    else
        return 0.5;
}

void Probabilistic_filter::coneRayTrace(double ox, double oy, double tx, double ty, double angle_std_dev)
{
    _max_angle = angle_std_dev;
    //ROS_INFO("angle std=%f",angle_std_dev);
    // calculate target props
    double dx = tx-ox, dy = ty-oy,
            theta = atan2(dy,dx), d = sqrt(dx*dx+dy*dy);
    // Integer Bounds of Update
    int bx0, by0, bx1, by1;

    // Bounds includes the origin
    _map->worldToMap(ox,oy,bx0,by0);
    bx1 = bx0;
    by1 = by0;
    // update bounds limits
    // Update Map with Target Point
    double mx, my;
    int a, b;

    // Update left side of sonar cone
    mx = ox + cos(theta-_max_angle) * d * 1.2;
    my = oy + sin(theta-_max_angle) * d * 1.2;
    _map->worldToMap(mx, my, a, b);
    bx0 = std::min(bx0, a);
    bx1 = std::max(bx1, a);
    by0 = std::min(by0, b);
    by1 = std::max(by1, b);

    // Update right side of sonar cone
    mx = ox + cos(theta+_max_angle) * d * 1.2;
    my = oy + sin(theta+_max_angle) * d * 1.2;
    _map->worldToMap(mx, my, a, b);
    bx0 = std::min(bx0, a);
    bx1 = std::max(bx1, a);
    by0 = std::min(by0, b);
    by1 = std::max(by1, b);

    // Limit Bounds to Grid
    bx0 = std::max(0, bx0);
    by0 = std::max(0, by0);

    bx1 = std::min((int)_map->sizeX(), bx1);
    by1 = std::min((int)_map->sizeY(), by1);
    //ROS_INFO("%i,%i,%i,%i",bx0,by0,bx1,by1);
    for(int x=bx0; x<=bx1; x++){
        for(int y=by0; y<=by1; y++){
            double wx, wy;
            _map->mapToWorld(x,y,wx,wy);
            //update_cell(ox, oy, theta, range, wx, wy, clear_sensor_cone);
            {
                int x_t, y_t;
                if(_map->worldToMap(wx, wy, x_t, y_t)){
                    double dx = wx-ox, dy = wy-oy;
                    double theta_t = atan2(dy, dx) - theta;
                    double theta_norm = angles::normalize_angle(theta_t); // theta -> [-pi,+pi]
                    double phi = sqrt(dx*dx+dy*dy);
#if SENSOR_MODEL_TYPE == KERNEL_MODEL
                   double sensor = gaussian_sensor_model(d,phi,theta_norm);
#elif SENSOR_MODEL_TYPE == CONE_MODEL
                   double sensor = kernel_sensor_model(d,phi,theta_norm);
#else
                    double sensor = 0;
#endif
                    double log_odds = std::log(sensor / (1 - sensor));
                    _map->editCell(x_t,y_t)->addMeasurement(log_odds);
                    //ROS_INFO("(%i,%i)=%f,%f", x,y,log_odds,sensor);
                }
            }
        }
    }
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
#if USE_RANGE_AND_NOISE_DECAY > 0
    const double dx = x1 - ori_x;
    const double dy = y1 - ori_y;
    const double dist = _map->resolution()*sqrt(dx*dx+dy*dy);
    return 1-std::min(2*_angle_std_dev*dist,1.0);
#else
    return 1;
#endif
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

        int pre_kernel_goal = x2 - (out_of_bounds || !markEnd ? RAY_END_BEFORE : _sensor_model.size()/2) * ix;

        while (x1 != pre_kernel_goal)
        {
            double weight = getRangeWeight(x1,y1,ori_x,ori_y);            
            // Mark Position clear
            try // remove to improve performance
            {
                _map->editCell(x1,y1)->addMeasurement(weight*_LOG_ODDS_FREE);
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
                _map->editCell(x1,y1)->addMeasurement( weight*(*sensor_ite));

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
            _map->editCell(x1,y1)->addMeasurement(weight*_LOG_ODDS_FREE);
        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));

        int pre_kernel_goal = y2 - (out_of_bounds || !markEnd ? RAY_END_BEFORE : _sensor_model.size()/2) * iy;

        while (y1 != pre_kernel_goal)
        {
            double weight = getRangeWeight(x1,y1,ori_x,ori_y);
            // Mark Position clear
            try // remove to improve performance
            {
                _map->editCell(x1,y1)->addMeasurement(weight*_LOG_ODDS_FREE);
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
                _map->editCell(x1,y1)->addMeasurement(weight*(*sensor_ite));

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
            _map->editCell(x1,y1)->addMeasurement(weight*_LOG_ODDS_FREE);
        }
    }
}



void Probabilistic_filter::resetEditLimits()
{
    _map->resetUpdateBounds();
}
