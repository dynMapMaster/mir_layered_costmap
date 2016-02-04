#ifndef PROBABILISTIC_FILTER_H
#define PROBABILISTIC_FILTER_H

#include <observation_interface.h>
#include <grid_structure.h>
#include <probablistic_cell.h>

class probabilistic_filter : public Observation_interface
{
public:
    probabilistic_filter(int xDim, int yDim, double resolution);
    ~probabilistic_filter();

    void raytrace(int x0, int y0, int x1, int y1, bool markEnd);
    double getOccupancyPrabability(int x, int y);
    void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);
    void resetUpdateLimits();
    void bresenham2D(int x1, int y1, const int x2, const int y2);

    // DEBUG
    Grid_structure<Probablistic_cell>* getMap(){return _map;}

private:
    Grid_structure<Probablistic_cell>* _map;
};

#endif // PROBABILISTIC_FILTER_H
