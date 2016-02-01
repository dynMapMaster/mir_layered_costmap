#include "pmac_cell.h"

Pmac_cell::Pmac_cell()
    : occupied_count(0), free_count(0), entry(0), exit(0),
    prev_occ_prob(0), previous_is_occupied(false)
{
}

void Pmac_cell::addProbability(double occ_prob)
{
    double free_prob = (1 - occ_prob);
    bool current_is_occupied = occ_prob > 0.5;
    if(current_is_occupied)
    {
        occupied_count += occ_prob;
        if(previous_is_occupied == false)
        {
            entry += 1 - prev_occ_prob;
        }
    }
    else
    {
        free_count += free_prob;
        if(previous_is_occupied)
        {
            exit += prev_occ_prob;
        }
    }
    previous_is_occupied = current_is_occupied;
    prev_occ_prob = occ_prob;
    // recency weightning
    if(current_is_occupied)
    {
        // Adjust active state
        if(occupied_count > MAX_NO_OF_OBS)
        {
            exit *= double(MAX_NO_OF_OBS) / occupied_count;
        }
        // Adjust inactive state
        free_count= 1 + (free_count-1) * recency_weightning;
        exit = 1 + (exit-1) * recency_weightning;
    }
    else
    {
        // Adjust active state
        if(free_count > MAX_NO_OF_OBS)
        {
            entry *= double(MAX_NO_OF_OBS) / free_count;
        }
        // Adjust inactive state
        occupied_count = 1 + (occupied_count-1) * recency_weightning;
        entry = 1 + (entry-1) * recency_weightning;
    }
}

double Pmac_cell::getLongTermOccupancyProb()
{
    double lambda_entry = (entry + 1) / (free_count + 1); // a(1,2)
    double lambda_exit = (exit + 1) / (occupied_count + 1); // a(2,1)
    return 1.0 / (lambda_exit + lambda_entry) * lambda_exit;
}
