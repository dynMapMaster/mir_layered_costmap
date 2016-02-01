#ifndef OBSERVATION_INTERFACE_H
#define OBSERVATION_INTERFACE_H


class Observation_interface
{
public:
    Observation_interface(){};
    virtual void raytrace(int x0, int y0, int x1, int y1, bool markEnd) = 0;
    virtual double getOccupancyPrabability(int x, int y) = 0;
    virtual void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax) = 0;
    virtual void resetEditLimits() = 0;
private:
};

#endif // OBSERVATION_INTERFACE_H

