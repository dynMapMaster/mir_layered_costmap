#ifndef PROBABLISTIC_CELL_H
#define PROBABLISTIC_CELL_H


class Probablistic_cell
{
public:
    const static int OBS_FREE = 0, OBS_OCCUPIED = 1;
    Probablistic_cell();
    void addMeasurement(int measurement, double prob);
    double getProbForOccupied();
private:
    double occupied_count, free_count;
};

#endif // PROBABLISTIC_CELL_H
