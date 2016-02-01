#include "pmac_learner.h"

Pmac_learner::Pmac_learner(int sizeX=2,int sizeY =2, double resolution=2)
    : grid(sizeX, sizeY, resolution)
{    
}

bool Pmac_learner::getCellValue(int x, int y, unsigned char& cellValueOutput)
{
    Pmac_cell* cell = grid.readCell(x,y);
    if (cell != NULL)
    {
        double occupancy_prob = cell->getLongTermOccupancyProb();
        if(occupancy_prob < 0) {
            occupancy_prob = 0;
        }
        else if(occupancy_prob > 1) {
            occupancy_prob = 1;
        }
        cellValueOutput = 255 * occupancy_prob;
    }
    else
    {
        return false;
    }
}

void Pmac_learner::addObservationMap(Observation_interface* observation)
{
    int max_x, max_y, min_y, min_x;
    grid.loadUpdateBounds( min_x,  max_x,  min_y,  max_y);
    if(max_x >= 0 && min_y >= 0 && max_y >= 0 && min_x >= 0)
    {
        for(int y = min_y; y <= max_y ; y++){
            for(int x = min_x; x <= max_x; x++){
                double occupancy_prob = observation->getOccupancyPrabability(x,y);
                if (occupancy_prob > 0) {
                    Pmac_cell* cell = grid.editCell(x,y);
                    cell->addProbability(occupancy_prob);
                }
            }
        }
    }

}

void Pmac_learner::loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax)
{
    grid.loadUpdateBounds(xMin, xMax, yMin, yMax);
}

double Pmac_learner::getOccupancyPrabability(int x, int y)
{
    Pmac_cell* cell = grid.readCell(x,y);
    if (cell != NULL)
        return cell->getLongTermOccupancyProb();
    else
        return -1;
}
