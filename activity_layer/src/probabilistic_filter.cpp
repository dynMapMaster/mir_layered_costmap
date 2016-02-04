#include "probabilistic_filter.h"
#include "Box.h"
#include <cmath>

class ray
{
public:
    ray(int xStart, int yStart, int xEnd, int yEnd);

    double x0, y0, dx, dy, dxInv, dyInv;
};

probabilistic_filter::probabilistic_filter(int xDim, int yDim, double resolution)
{
    _map = new Grid_structure<Probablistic_cell>(xDim,yDim,resolution);
}

probabilistic_filter::~probabilistic_filter()
{
    delete _map;
}


void probabilistic_filter::raytrace(int x0, int y0, int x1, int y1, bool markEnd)
{
    bresenham2D(x0,y0,x1,y1);
    if(markEnd)
          _map->editCell(x1,y1)->addMeasurement(1,1);
}


double probabilistic_filter::getOccupancyPrabability(int x, int y)
{
    Probablistic_cell* cell = _map->readCell(x,y);
    double result = -1;
    if(cell != NULL)
    {
        result = cell->getProbForOccupied();
    }
    return result;
}


inline void probabilistic_filter::bresenham2D(int x1, int y1, const int x2, const int y2)
{
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) *2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;

    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2)
        {
            _map->editCell(x1,y1)->addMeasurement(0,1);
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
        // error may go below zero
        int error(delta_x - (delta_y >> 1));

        while (y1 != y2)
        {
            _map->editCell(x1,y1)->addMeasurement(0,1);
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
}


void probabilistic_filter::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    _map->loadUpdateBounds(xMin,xMax,yMin,yMax);
}



