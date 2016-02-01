#ifndef PMAC_LEARNER_H
#define PMAC_LEARNER_H
#include <pmac_cell.h>
#include <grid_structure.h>
#include <costmap_interpretator.h>
#include <observation_interface.h>

class Pmac_learner : public Costmap_interpretator
{
public:
    Pmac_learner(int sizeX, int sizeY, double resolution);
    bool getCellValue(int x, int y, unsigned char &cellValueOutput);
    void addObservationMap(Observation_interface* observation);
    void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax) = 0;
    double getOccupancyPrabability(int x, int y); // just for testing
private:    
    Grid_structure<Pmac_cell> grid;
};

#endif // PMAC_LEARNER_H
