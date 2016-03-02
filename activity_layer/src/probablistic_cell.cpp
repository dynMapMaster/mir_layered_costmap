#include "probablistic_cell.h"
#include <cmath>
Probablistic_cell::Probablistic_cell()
    : log_odds(0), hit_count(0)
{
}

void Probablistic_cell::addMeasurement(double log_odds_update)
{
    if(std::abs(log_odds_update) > 0.0001)
        ++hit_count;
    log_odds += log_odds_update;
}

double Probablistic_cell::getProbForOccupied(const bool reset)
{    
    double adjusted_odds = 0;
    if(reset)
        adjusted_odds = log_odds;
    else if(hit_count > 0){
        adjusted_odds = log_odds / hit_count;
    }

    double occupied_prob = 1 - 1 / (1 + std::exp(adjusted_odds));
    if(reset)
    {
        log_odds = 0;
        hit_count = 0;
    }
    return occupied_prob;
}

void Probablistic_cell::resetCell()
{
    log_odds = 0;
}
