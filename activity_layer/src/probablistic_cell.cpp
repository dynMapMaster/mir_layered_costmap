#include "probablistic_cell.h"
#include <cmath>
#include <ros/ros.h>

Probablistic_cell::Probablistic_cell()
    : log_odds(0)
{
}

void Probablistic_cell::addMeasurement(double log_odds_update)
{
        log_odds += log_odds_update;
        if(log_odds_update < 0)
        {
            current_consecutive_free++;
            if(current_consecutive_free > max_consecutive_free)
                max_consecutive_free = current_consecutive_free;
        }
        else
        {
            current_consecutive_free = 0;
        }
}

double Probablistic_cell::getProbForOccupied(const bool reset)
{
    double occupied_prob = 1 - 1 / (1 + std::exp(log_odds));

    if(reset)
    {
        if(max_consecutive_free > 15)
        {
            //ROS_ERROR("HIGHLY DYNAMIC");
           // occupied_prob = 0;
        }
        resetCell();
    }
    return occupied_prob;
}

void Probablistic_cell::resetCell()
{
    log_odds = 0;
    max_consecutive_free = 0;
    current_consecutive_free = 0;
}

bool Probablistic_cell::deserialize(const std::vector<double>& values)
{
    bool returnVal = false;
    if(values.size() == 1)
    {
        log_odds = 1 - 1 / (1 + std::exp(values[0]));
        returnVal = true;
    }
    return returnVal;
}

std::vector<double> Probablistic_cell::serialize()
{
    std::vector<double> result(1);
    result[0] = getProbForOccupied(false);
    return result;
}
