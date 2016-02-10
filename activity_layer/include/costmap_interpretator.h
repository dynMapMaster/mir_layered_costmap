#ifndef COSTMAP_INTERPRETATOR_H
#define COSTMAP_INTERPRETATOR_H
#include <observation_interface.h>

class Costmap_interpretator
{
public:
    enum Initial_values {Free, Unknown, Obstacle};


    Costmap_interpretator() {};
    virtual bool getCellValue(int x, int y, unsigned char & cellValueOutput) = 0;
    virtual void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax) = 0;
    virtual void addObservationMap(Observation_interface* observation) = 0;
    virtual void resetEditLimits() = 0;
    virtual void initCell(int x, int y, Initial_values value) = 0;


private:

};

#endif // COSTMAP_INTERPRETATOR_H
