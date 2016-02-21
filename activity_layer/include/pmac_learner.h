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
    void setUpdateInterval(double time_between_updates);
    void initCell(int x, int y, Initial_values value);
    std::vector<std::vector<double> > serialize();
    void deserialize(const std::vector<std::vector<double> > &values);

    // Constants
    static const uint64_t UPDATE_INTERVAL = 30e9;           // Time before new observations are accepted, in nanoseconds
    static const double MIN_OBS_VALUE = 40;                 // Minimum sum of observations before cost is calculated
    static const unsigned char OBSTACLE_THRESHOLD = 200;    // Equal or above this is obstacles
private:    
    Grid_structure<Pmac_cell> grid;
    uint64_t update_time;
    void translateOcc(unsigned char &value);
};

#endif // PMAC_LEARNER_H
