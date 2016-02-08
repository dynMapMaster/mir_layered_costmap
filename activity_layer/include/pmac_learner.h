#ifndef PMAC_LEARNER_H
#define PMAC_LEARNER_H
#include <pmac_cell.h>
#include <grid_structure.h>
#include <costmap_interpretator.h>
#include <observation_interface.h>
#include <ros/ros.h>
class Pmac_learner : public Costmap_interpretator
{
public:
    Pmac_learner(int sizeX, int sizeY, double resolution);
    bool getCellValue(int x, int y, unsigned char &cellValueOutput);
    void addObservationMap(Observation_interface* observation);
    void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);
    double getOccupancyPrabability(int x, int y); // just for testing    
    void resetEditLimits();
    static const uint64_t UPDATE_INTERVAL = 5e9; // in nanoseconds
    void setUpdateInterval(double time_between_updates);
private:    
    Grid_structure<Pmac_cell> grid;
    double update_time;
};

#endif // PMAC_LEARNER_H
