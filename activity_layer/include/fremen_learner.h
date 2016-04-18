#ifndef FREMEN_LEARNER_H
#define FREMEN_LEARNER_H

#include <costmap_interpretator.h>
#include <grid_structure.h>
#include <observation_interface.h>
#include <fremen_cell.h>
#include <ros/ros.h>

class Fremen_learner : public Costmap_interpretator
{
public:
    Fremen_learner(int, int, double);
    virtual bool getCellValue(int x, int y, unsigned char & cellValueOutput);
    virtual void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);
    virtual void addObservationMap(Observation_interface* observation);
    virtual void resetEditLimits();
    virtual double getPredictScore();
    virtual void initCell(int x, int y, Initial_values value);
    virtual void deserialize(const std::vector<std::vector<double> >& values);
    virtual std::vector<std::vector<double> > serialize();

private:
    static constexpr uint64_t UPDATE_INTERVAL = 0e9;
    static constexpr double MIN_OBS_VALUE = 1; // Minimum sum of observations before cost is calculated
    static constexpr unsigned char OBSTACLE_THRESHOLD = 200;
    ros::Publisher currentErrorPub;
    Grid_structure<Fremen_cell> grid;

    uint64_t scored_observations;
    uint64_t update_time;
    double sse_score;

    double getOccupancyPrabability(int x, int y);
    void translateOcc(unsigned char &value);
};

#endif // FREMEN_LEARNER_H
