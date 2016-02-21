#include "probablistic_cell.h"
#include <cmath>
Probablistic_cell::Probablistic_cell()
    : log_odds(0)
{
}

inline void Probablistic_cell::addMeasurement(double log_odds_update)
{
    log_odds += log_odds_update;
}

double Probablistic_cell::getProbForOccupied()
{
    double occupied_prob = 1 - 1 / (1 + std::exp(log_odds));
    log_odds = 0;
    return occupied_prob;
}

void Probablistic_cell::resetCell()
{
    log_odds = 0;
}
