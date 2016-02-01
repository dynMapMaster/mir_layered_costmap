#include "probablistic_filter.h"

Probablistic_filter::Probablistic_filter()
{

}

void Probablistic_filter::traceLine(int x0, int y0, int x1, int y1, bool markEnd)
{
   unsigned long currentTime = ros::Time::now().toSec() * 1000;
   //ROS_INFO("Tracing line from (%i,%i) to (%i,%i)",x0,y0,x1,y1);
   bresenham2D(x0,y0,x1,y1);
   if(markEnd)
        pointObservedOccupied(x1,y1,currentTime);
}

inline void Probablistic_filter::bresenham2D(int x1, int y1, const int x2, const int y2)
{
    unsigned long currentTime = ros::Time::now().toSec() * 1000;
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
            //ROS_INFO("Clearing point (%i,%i)",x1,y1);
            pointObservedFree(x1,y1,currentTime);
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
            //ROS_INFO("Clearing point (%i,%i)",x1,y1);
           pointObservedFree(x1,y1,currentTime);
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

void Probablistic_filter::pointObservedOccupied(int x, int y)
{
    getDataPointer(x,y)->observedOccupied();
    this->updateEditLimits(x,y);
}

void Probablistic_filter::addObservationMap(activityMap* newMap, collapsingMethod method, int minUnobservedTimeMs)
{
    if(newMap->_editMaxX >= 0 && newMap->_editMinY >= 0 && newMap->_editMaxY >= 0 && newMap->_editMinX >= 0)
    {
        int newXmin = -1 , newXmax =-1, newYmin = -1 , newYmax = -1;
        for(int y = newMap->_editMinY; y <= newMap->_editMaxY ; y++){
            for(int x = newMap->_editMinX; x <= newMap->_editMaxX; x++){
                if(newMap->pointContainsData(x,y))
                {
                    activityMapComponent* newMapCell = newMap->getDataPointer(x,y);
                    if(currentTime - newMapCell->getTimestamp() > minUnobservedTimeMs )
                    {
                        this->getDataPointer(x,y)->add(newMapCell,method);
                        newMapCell->resetCell();
                    }
                    else
                    {
                        // set new limits
                        if(newXmin < 0 || x < newXmin)
                            newXmin = x;
                        if(newXmax < 0 || x > newXmax)
                            newXmax = x;
                        if(newYmin < 0 || y < newYmin)
                            newYmin = y;
                        if(newYmax < 0 || y > newYmax)
                            newYmax = y;
                    }
                }
                this->updateEditLimits(x,y);
            }
        }
        //newMap->resetEditLimits();
        newMap->setEditLimits(newXmin,newXmax,newYmin, newYmax);
    }
}
