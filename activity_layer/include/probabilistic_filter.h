#ifndef PROBABILISTIC_FILTER_H
#define PROBABILISTIC_FILTER_H

#include <observation_interface.h>
#include <grid_structure.h>
#include <probablistic_cell.h>
#include <vector>
#include <utility>

class Probabilistic_filter : public Observation_interface
{
public:
    Probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev, double origin_x, double origin_y);
    ~Probabilistic_filter();

    // Traces a ray to x1,y1. All positions are written with a gaussian kernel based on lasernoise
    void raytrace(int x0, int y0, int x1, int y1, bool markEnd);

    // return the occupancy probability of the underlying cell
    double getOccupancyPrabability(int x, int y);

    // Loads the update bounds from grid structure
    void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);

    // Resets the grid structures update limits    
    void resetEditLimits();
    // raytrace with a sonar model
    void coneRayTrace(double ox, double oy, double tx, double ty, double angle_std_dev, bool mark_end);
    double _angle_std_dev;
    double _y_std_dev, _x_std_dev;
private:
    Grid_structure<Probablistic_cell>* _map;
    double _laser_noise_var;
    double _laser_noise_std_dev;

    bool enforceBounds(int& x, int& y);

    std::vector<double> _sensor_model;
    constexpr static double _LOG_ODDS_FREE = -0.4055;

    constexpr static double _RAYTRACE_INSERT_TRESHOLD = 0.00001;

    int _sensor_model_occupancy_goal_index;

    inline void bresenham2Dv0(int x1, int y1, int x2, int y2, bool markEnd = true);
    inline double getRangeWeight(int x1, int y1, int ori_x, int ori_y);

};

#endif // PROBABILISTIC_FILTER_H
