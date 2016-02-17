#include "probabilistic_filter.h"
#include <cmath>


probabilistic_filter::probabilistic_filter(int xDim, int yDim, double resolution, double laserStdDev)
{
    _map = new Grid_structure<Probablistic_cell>(xDim,yDim,resolution);
    _laserNoiseVar = laserStdDev * laserStdDev;
    _laserNoiseStdDev = laserStdDev;


    // Setup sensormodel lookup table
    sensorModelOccupancy.push_back(0);
    sensorModelOccupancy.push_back(0);
    sensorModelOccupancy.push_back(0);
    sensorModelOccupancy.push_back(1);


    sensorModelOccupancyGoalIndex = 3;
}

probabilistic_filter::~probabilistic_filter()
{
    delete _map;
}

void probabilistic_filter::raytrace(int x0, int y0, int x1, int y1, bool markEnd)
{   
    std::vector<std::pair<int, int> > resultLine = bresenham2Dv0(x0, y0, x1, y1);
    std::pair<int, int> goal(x1,y1);
    int goalIndex = resultLine.size();
    double dx= x1-x0;
    double dy = y1 - y0;
    double totalDelta = sqrt(dx*dx+dy*dy);
    if(totalDelta > 0)
    {
        dx /= totalDelta;
        dy /= totalDelta;
        int xExtended = x1 + dx * 3 * (_laserNoiseStdDev / _map->resolution());
        int yExtended = y1 + dy * 3 * (_laserNoiseStdDev / _map->resolution());
       // std::vector<std::pair<int, int> > extendedLine = bresenham2Dv0(x1, y1, xExtended, yExtended);
       // resultLine.insert(resultLine.end(),extendedLine.begin(), extendedLine.end());
    }

    bool goalEncountered = false;
    for(int i = 0; i < (int) resultLine.size();i++)
    {
        int cellsToGoal = goalIndex - i;

        // lookup occupancy value
        double occValue = lookUpProbabilityFromSensorModel(cellsToGoal);

            // mark occupied

            try
            {
                if(resultLine[i].first >= 0 && resultLine[i].first < _map->sizeX()  && resultLine[i].second >= 0 && resultLine[i].second < _map->sizeY() )
                {
                    if(markEnd)
                    {
                        _map->editCell(resultLine[i].first,resultLine[i].second)->addMeasurement(Probablistic_cell::OBS_OCCUPIED,occValue);
                    }
                     _map->editCell(resultLine[i].first,resultLine[i].second)->addMeasurement(Probablistic_cell::OBS_FREE,1-occValue);
                }

            }
            catch(const char* s)
            {
                ROS_ERROR("Filter marking goal error %s", s);
            }
    }

}

double probabilistic_filter::lookUpProbabilityFromSensorModel(int relativeToGoal)
{
    if(relativeToGoal + sensorModelOccupancyGoalIndex >= 0 && relativeToGoal + sensorModelOccupancyGoalIndex < sensorModelOccupancy.size())
    {
        return sensorModelOccupancy[relativeToGoal + sensorModelOccupancyGoalIndex];
    }
    else
    {
        return 0;
    }
}

double probabilistic_filter::getOccupancyPrabability(int x, int y)
{
    Probablistic_cell* cell = _map->readCell(x,y);
    double result = -1;
    if(cell != NULL)
    {
        result = cell->getProbForOccupied();
        cell->resetCell();
    }
    return result;
}

double probabilistic_filter::calculateProb(const std::vector<double>& origin, const std::vector<double>& direction, const std::vector<double>& intersectResult, int goalX, int goalY)
{
    double firstX = (goalX+0.5) - (origin[0] + direction[0] * intersectResult[0]);
    double firstY = (goalY+0.5) - (origin[1] + direction[1] * intersectResult[0]);
    double firstDist = sqrt(firstX*firstX + firstY * firstY);

    double secondX = (goalX+0.5) - (origin[0] + direction[0] * intersectResult[1]);
    double secondY = (goalY+0.5) - (origin[1] + direction[1] * intersectResult[1]);
    double secondDist = sqrt(secondX*secondX + secondY * secondY);
    if((firstX < 0 && secondX >= 0) || ((firstX >= 0 && secondX < 0)))
    {
        secondDist *= -1;
    }


    double v1 = phi(secondDist / (_laserNoiseVar));
    double v2 = phi(firstDist / (_laserNoiseVar));

    return std::abs(v2 - v1);
}

void probabilistic_filter::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    _map->loadUpdateBounds(xMin,xMax,yMin,yMax);
}


double probabilistic_filter::phi(double x)
{
    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}





inline std::vector<std::pair<int,int> > probabilistic_filter::bresenham2Dv0(int x1, int y1, const int x2, const int y2)
{    
    int delta_x(x2 - x1);
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) *2;

    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) * 2;
    std::vector<std::pair<int, int> > resultLine;

    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));

        while (x1 != x2)
        {
            std::pair<int,int> n(x1,y1);
            resultLine.push_back(n);

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

            std::pair<int,int> n(x1,y1);
            resultLine.push_back(n);

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

    return resultLine;
}

void probabilistic_filter::resetEditLimits()
{
    _map->resetUpdateBounds();
}

/*void raytrace(int x0, int y0, int x1, int y1, bool markEnd)
{
    std::vector<std::pair<int, int> > resultLine = bresenham2Dv0(x0, y0, x1, y1);
    std::pair<int, int> goal(x1,y1);
    double dx= x1-x0;
    double dy = y1 - y0;
    double totalDelta = sqrt(dx*dx+dy*dy);
    if(totalDelta > 0)
    {
        dx /= totalDelta;
        dy /= totalDelta;
        int xExtended = x1 + dx * 6;
        int yExtended = y1 + dy * 6;
        std::vector<std::pair<int, int> > extendedLine = bresenham2Dv0(x1, y1, xExtended, yExtended);
        resultLine.insert(resultLine.end(),extendedLine.begin(), extendedLine.end());
    }

    bool goalEncountered = false;
    for(int i = 0; i < (int) resultLine.size();i++)
    {
        for(int k = -2; k < 3;k++)
        {
            if(i+k >= 0 && i+k < (int) resultLine.size())
            {
                double distToCurrent = sqrt((resultLine[i+k].first - resultLine[i].first)*(resultLine[i+k].first - resultLine[i].first)+(resultLine[i+k].second - resultLine[i].second)*(resultLine[i+k].second - resultLine[i].second));
                double leftBound = (distToCurrent - 0.5) / _laserNoiseVar;
                double rightBound = (distToCurrent + 0.5) / _laserNoiseVar;
                double prob = std::abs(phi(rightBound) - phi(leftBound));

                if(resultLine[i].first == x1 && resultLine[i].second == y1)
                {
                    if(markEnd)
                    {
                        try
                        {
                            if(resultLine[i+k].first >= 0 && resultLine[i+k].first < _map->sizeX()  && resultLine[i+k].second >= 0 && resultLine[i+k].second < _map->sizeY() )
                            {
                                 _map->editCell(resultLine[i+k].first,resultLine[i+k].second)->addMeasurement(Probablistic_cell::OBS_OCCUPIED,prob);
                            }
                        }
                        catch(const char* s)
                        {
                            ROS_ERROR("Filter marking goal error %s", s);
                        }
                    }
                    goalEncountered = true;
                }
                else
                {
                    try
                    {
                        if(resultLine[i+k].first >= 0 && resultLine[i+k].first < _map->sizeX()  && resultLine[i+k].second >= 0 && resultLine[i+k].second < _map->sizeY() )
                        {
                            _map->editCell(resultLine[i+k].first,resultLine[i+k].second)->addMeasurement(Probablistic_cell::OBS_FREE,prob);
                        }
                    }
                    catch(const char* s)
                    {
                        ROS_ERROR("Filter marking free error %s", s);
                    }
                }

            }
        }
        if(goalEncountered)
            break;
    }

}
*/
