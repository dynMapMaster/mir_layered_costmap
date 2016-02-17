#ifndef PROBABILISTIC_FILTER_H
#define PROBABILISTIC_FILTER_H

#include <observation_interface.h>
#include <grid_structure.h>
#include <probablistic_cell.h>
#include <vector>
#include <utility>


class probabilistic_filter : public Observation_interface
{
public:
    probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev);
    ~probabilistic_filter();

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
    double _laserNoiseVar;
    double _laserNoiseStdDev;

    std::pair<double, double> lookUpProbabilityFromSensorModel(int relativeToGoal);
    std::vector<double> sensorModelOccupancy, sensorModelFree;
    int sensorModelOccupancyGoalIndex;

    double calculateProb(const std::vector<double>& origin, const std::vector<double>& direction, const std::vector<double>& intersectResult, int goalX, int goalY);
    std::vector<std::pair<int,int> > bresenham2Dv0(int x1, int y1, const int x2, const int y2);
    double phi(double x);
};

#endif // PROBABILISTIC_FILTER_H
