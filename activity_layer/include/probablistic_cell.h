#ifndef PROBABLISTIC_CELL_H
#define PROBABLISTIC_CELL_H


class Probablistic_cell
{
public:
    Probablistic_cell();
    void addMeasurement(double log_odds_update);
    double getProbForOccupied(const bool reset=true);
    void resetCell();
private:
    double log_odds;
};

#endif // PROBABLISTIC_CELL_H
