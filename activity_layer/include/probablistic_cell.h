#ifndef PROBABLISTIC_CELL_H
#define PROBABLISTIC_CELL_H
#include <vector>

class Probablistic_cell
{
public:
    Probablistic_cell();
    void addMeasurement(double log_odds_update, bool override=false);
    double getProbForOccupied(const bool reset=true);
    void resetCell();
    bool deserialize(const std::vector<double>& values);
    std::vector<double> serialize();
private:
    double log_odds;
};

#endif // PROBABLISTIC_CELL_H
