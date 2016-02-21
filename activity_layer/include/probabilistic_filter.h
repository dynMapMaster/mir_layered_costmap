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
    Probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev);
    ~Probabilistic_filter();

    // Traces a ray to x1,y1. All positions are written with a gaussian kernel based on lasernoise
    void raytrace(int x0, int y0, int x1, int y1, bool markEnd);

    // return the occupancy probability of the underlying cell
    double getOccupancyPrabability(int x, int y);

    // Loads the update bounds from grid structure
    void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);

    // Resets the grid structures update limits    
    void resetEditLimits();

private:
    Grid_structure<Probablistic_cell>* _map;
    double _laser_noise_var;
    double _laser_noise_std_dev;

    double lookUpProbabilityFromSensorModel(int relativeToGoal);
    bool enforceBounds(int& x, int& y);
    //const size_t sensor_model_size = 5;
    std::vector<double> _sensor_model;
    const static double _LOG_ODDS_FREE = -11.5129354649202;
    int _sensor_model_occupancy_goal_index;

    double calculateProb(const std::vector<double>& origin, const std::vector<double>& direction, const std::vector<double>& intersectResult, int goalX, int goalY);
    inline void bresenham2Dv0(int x1, int y1, int x2, int y2, bool markEnd = true);
    double phi(double x);
};

#endif // PROBABILISTIC_FILTER_H
