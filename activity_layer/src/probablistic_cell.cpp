#include "probablistic_cell.h"
#include <cmath>
#include <ros/ros.h>
Probablistic_cell::Probablistic_cell()
    : log_odds(0)
{
}

void Probablistic_cell::addMeasurement(double log_odds_update,bool override)
{
    if(override)
    {
        if(std::abs(log_odds_update) > std::abs(log_odds))
            log_odds = log_odds_update;
    }
    else
        log_odds += log_odds_update;
}

double Probablistic_cell::getProbForOccupied(const bool reset)
{
    double occupied_prob = 1 - 1 / (1 + std::exp(log_odds));
    if(reset)
    {
        log_odds = 0;
    }
    return occupied_prob;
}

void Probablistic_cell::resetCell()
{
    log_odds = 0;
}
