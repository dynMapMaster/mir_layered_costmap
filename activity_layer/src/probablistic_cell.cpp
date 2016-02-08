#include "probablistic_cell.h"

Probablistic_cell::Probablistic_cell()
    : occupied_count(0), free_count(0)
{

}

void Probablistic_cell::addMeasurement(int measurement, double prob)
{
    if(measurement == Probablistic_cell::OBS_FREE)
    {
        free_count += prob;
    }
    else if (measurement == Probablistic_cell::OBS_OCCUPIED)
    {
        occupied_count += prob;
    }
}

double Probablistic_cell::getProbForOccupied()
{
    double denum = occupied_count + free_count;
    double occupiedness;
    if(denum > 0)
    {
        occupiedness = occupied_count / denum;
    }
    else
    {
        occupiedness = -1;
    }
    free_count = 0;
    occupied_count = 0;
    return occupiedness;
}

void Probablistic_cell::resetCell()
{
    occupied_count = 0;
    free_count = 0;
}
