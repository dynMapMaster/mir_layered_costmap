#ifndef BAYES_LEARNER_H
#define BAYES_LEARNER_H
#include "costmap_interpretator.h"
#include "grid_structure.h"
#include "probablistic_cell.h"

class Bayes_learner : public Costmap_interpretator
{
public:
    Bayes_learner();
    Bayes_learner(int size_x, int size_y, double resolution);
    virtual bool getCellValue(int x, int y, unsigned char & cellValueOutput);
    virtual void loadUpdateBounds(int& xMin, int& xMax, int& yMin, int& yMax);
    virtual void addObservationMap(Observation_interface* observation);
    virtual void resetEditLimits();
    virtual void initCell(int x, int y, Initial_values value);

    static const uint64_t UPDATE_INTERVAL = 30e9;           // Time before new observations are accepted, in nanoseconds
    const static double _LOG_ODDS_FREE = -11.5129354649202;
    const static double _INITIAL_OCCUPIED_PORBABILITY = 0.9;
private:
    Grid_structure<Probablistic_cell> _grid;
    uint64_t update_time;
};

#endif // BAYES_LEARNER_H
