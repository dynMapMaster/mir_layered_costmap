#include "pmac_cell.h"

#include "boost/numeric/ublas/matrix.hpp"
#include <cmath>
#include <float.h>


Pmac_cell::Pmac_cell()
    : occupied_count(DBL_MIN), free_count(DBL_MIN), entry(DBL_MIN), exit(DBL_MIN),
      prev_occ_prob(0.5), lastObservedTime(0.0), last_exit_cnt(DBL_MIN), last_entry_cnt(DBL_MIN),
      old_entry_cnt(DBL_MIN), old_exit_cnt(DBL_MIN), last_entry_residue(0.0), last_exit_residue(0.0),
      old_exit_residue(0.0), old_entry_residue(0.0), estimated_occupancy_prob(0.5)
{
}

void Pmac_cell::addProbability(double occ_prob, double timeStamp)
{
    lastObservedTime = timeStamp;

    if(occ_prob > 0.5)
    {
        double occupied_prob = (occ_prob - 0.5) * 2;
        occupied_count += occupied_prob;
        // occ state
        if(prev_occ_prob <= 0.5)
        {
            last_exit_residue = 0;
            last_exit_cnt = 0;
            double old_entry_residue_avg = old_entry_residue / old_entry_cnt;
            double current_entry_residue_avg = last_entry_residue / last_entry_cnt;

            if(current_entry_residue_avg > old_entry_residue_avg)
            {
                last_entry_residue = current_entry_residue_avg;
                old_entry_residue = 0.0;
                old_entry_cnt = 0.0;
            }
            else {
                last_entry_residue = old_entry_residue_avg;
            }
            last_transition = timeStamp;
        }

        // start counting the exit max up
        last_exit_residue += occupied_prob;
        ++last_exit_cnt;
        old_exit_residue += occupied_prob;
        ++old_exit_cnt;

        // increment entry event
        if(last_entry_residue > 0.0)
        {
            double next_entry_cnt = std::min(occupied_prob,last_entry_residue);
            entry += next_entry_cnt;
            last_entry_residue -= next_entry_cnt;
            old_entry_residue -= next_entry_cnt;
        }
    }
    else
    {
        double free_prob = (0.5 - occ_prob) * 2;
        free_count += free_prob;

        if(prev_occ_prob > 0.5)
        {
            last_entry_residue = 0;
            last_entry_cnt = 0;
            double old_exit_residue_avg = old_exit_residue / old_exit_cnt;
            double current_exit_residue_avg = last_exit_residue / last_exit_cnt;

            if(current_exit_residue_avg > old_exit_residue_avg)
            {
                last_exit_residue = current_exit_residue_avg;
                old_exit_residue = 0.0;
                old_exit_cnt = 0.0;
            }
            else {
                last_exit_residue = old_exit_residue_avg;
            }
            last_transition = timeStamp;
        }
        last_entry_residue += free_prob;
        ++last_entry_cnt;
        old_entry_residue += free_prob;
        ++old_entry_cnt;
        if(last_exit_residue > 0.0)
        {
            double next_exit_cnt = std::min(free_prob,last_exit_residue);
            exit += next_exit_cnt;
            last_exit_residue -= next_exit_cnt;
            old_exit_residue -= next_exit_cnt;
        }
    }
    prev_occ_prob = occ_prob;

    /*
    bool current_is_occupied = occ_prob > 0.5;
    occ_prob = occ_prob > 0.5 ? 1 : 0; // pmac -> imac
    double free_prob = (1 - occ_prob);
    if(current_is_occupied)
    {
        occupied_count += occ_prob;
        if(previous_is_occupied == false)
        {
            entry += (1 - prev_occ_prob);
            last_transition = timeStamp;
        }
    }
    else
    {
        free_count += free_prob;
        if(previous_is_occupied)
        {
            exit += prev_occ_prob;
            last_transition = timeStamp;
        }
    }
    previous_is_occupied = current_is_occupied;
    prev_occ_prob = occ_prob;
    /*
    // recency weightning
    if(current_is_occupied)
    {
        // Adjust active state
        if(occupied_count > MAX_NO_OF_OBS)
        {
            exit *= double(MAX_NO_OF_OBS) / occupied_count;
            occupied_count = MAX_NO_OF_OBS;
        }
        // Adjust inactive state
        free_count= 1 + (free_count-1) * FORGET_FACTOR;
        exit = 1 + (exit-1) * FORGET_FACTOR;
    }
    else
    {
        // Adjust active state
        if(free_count > MAX_NO_OF_OBS)
        {
            entry *= double(MAX_NO_OF_OBS) / free_count;
            free_count = MAX_NO_OF_OBS;
        }
        // Adjust inactive state
        occupied_count = 1 + (occupied_count-1) * FORGET_FACTOR;
        entry = 1 + (entry-1) * FORGET_FACTOR;
    }
    */

    // Update estimated occupancy probability

    //correct
    estimated_occupancy_prob += 2*std::abs(occ_prob - 0.5)*(occ_prob - estimated_occupancy_prob);
}

double Pmac_cell::getLambdaExit()
{
    return exit / occupied_count; // a(2,1)
}

double Pmac_cell::getLambdaEntry()
{
    return entry / free_count; // a(1,2)
}

double Pmac_cell::getLongTermOccupancyProb()
{
    double lambda_entry = entry / free_count; // a(1,2)
    double lambda_exit = exit / occupied_count; // a(2,1)
    return 1.0 / (lambda_exit + lambda_entry) * lambda_entry;
}

double Pmac_cell::getProjectedOccupancyProbability(unsigned noOfProjections)
{
    //predict

    double lambda_entry = entry / free_count; // a(1,2)
    double lambda_exit = exit / occupied_count; // a(2,1)
    for (int i = 0; i < noOfProjections; ++i) {
        estimated_occupancy_prob *= ((1-lambda_exit) + lambda_entry);
    }
    return estimated_occupancy_prob;

    boost::numeric::ublas::matrix<double> marchovM(2,2);
    marchovM(0,0) = 1-lambda_exit;
    marchovM(0,1) = lambda_exit;
    marchovM(1,0) = lambda_entry;
    marchovM(1,1) = 1 - lambda_entry;

    boost::numeric::ublas::matrix<double> states(1,2);
    states(0,0) = 0.5;//prev_occ_prob;
    states(0,1) = 0.5;//1 - prev_occ_prob;

    for(unsigned i = 0; i < noOfProjections; i++){
        states = boost::numeric::ublas::prod(states, marchovM);
    }
    return states(0,0);
}

double Pmac_cell::getTemporalPrediction(int forward_steps)
{
    /*
    double lambda_entry = (entry + 1) / (free_count + 1); // a(1,2)
    double lambda_exit = (exit + 1) / (occupied_count + 1); // a(2,1)

    lambda_entry = (lambda_entry > 1 ? 1 : lambda_entry);
    lambda_exit = (lambda_exit > 1 ? 1 : lambda_exit);

    int free_time = 1.0 / (1.0- lambda_entry);
    int occ_time = 1.0 / (1.0-lambda_exit);
    double weight = forward_steps / (free_time+occ_time);
    forward_steps %= (free_time+occ_time);
    double occ_value = 0;
    if(previous_is_occupied)
    {
        if(forward_steps > occ_time)
            occ_value = 0;
        else
            occ_value = 1;
    }
    else
    {
        if(occupied_count > 2)
        {
            if(forward_steps > free_time)
                occ_value = 1;
            else
                occ_value = 0;
        }
        else
            occ_value = 0;
    }

    occ_value *= 1.0/weight;

    return occ_value;
    */
    return -1;
}

double Pmac_cell::getLastTransitionTime()
{
    return last_transition;
}

void Pmac_cell::init(double initialOccupancy, double initialFree)
{
    if(initialOccupancy > initialFree)
    {
        //occupied_count += initialOccupancy;
        occupied_count = DBL_MIN;
        prev_occ_prob = 0.5;
    }
    else
    {
        //free_count += initialFree;
        free_count = DBL_MIN;
        prev_occ_prob = 0.5;
    }
}

double Pmac_cell::observationSum()
{
    return free_count + occupied_count;
}

double Pmac_cell::getLastObservationTime()
{
    return lastObservedTime;
}

unsigned Pmac_cell::getMixingTime()
{
    double lambda_entry = entry / free_count; // a(1,2)
    double lambda_exit = exit / occupied_count; // a(2,1)

    return std::log(0.001*(lambda_entry/(lambda_entry+lambda_exit))) / (log(std::fabs(1-lambda_exit-lambda_entry)));
}

bool Pmac_cell::deserialize(const std::vector<double>& values)
{
    bool returnVal = false;
    if(values.size() == 5)
    {
        occupied_count = values[0];
        free_count = values[1];
        entry = values[2];
        exit = values[3];
        prev_occ_prob = values[4];
        lastObservedTime = 0;
        returnVal = true;
    }
    return returnVal;
}

std::vector<double> Pmac_cell::serialize()
{
    std::vector<double> result(5);
    result[0] = occupied_count;
    result[1] = free_count;
    result[2] = entry;
    result[3] = exit;
    result[4] = prev_occ_prob;
    return result;
}
