#include "pmac_cell.h"

#include "boost/numeric/ublas/matrix.hpp"
#include <cmath>


Pmac_cell::Pmac_cell()
    : occupied_count(0), free_count(0), entry(0), exit(0),
      prev_occ_prob(0.5), previous_is_occupied(false), lastObservedTime(0.0)
{
}

void Pmac_cell::addProbability(double occ_prob, double timeStamp)
{
    double free_prob = (1 - occ_prob);
    bool current_is_occupied = occ_prob > 0.5;
    lastObservedTime = timeStamp;
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
}

double Pmac_cell::getLongTermOccupancyProb()
{
    double lambda_entry = (entry + 1) / (free_count + 1); // a(1,2)
    double lambda_exit = (exit + 1) / (occupied_count + 1); // a(2,1)
    return 1.0 / (lambda_exit + lambda_entry) * lambda_entry;
}

double Pmac_cell::getProjectedOccupancyProbability(unsigned noOfProjections)
{
    double lambda_entry = (entry + 1) / (free_count + 1); // a(1,2)
    double lambda_exit = (exit + 1) / (occupied_count + 1); // a(2,1)

    boost::numeric::ublas::matrix<double> marchovM(2,2);
    marchovM(0,0) = 1-lambda_exit;
    marchovM(0,1) = lambda_exit;
    marchovM(1,0) = lambda_entry;
    marchovM(1,1) = 1 - lambda_entry;

    boost::numeric::ublas::matrix<double> states(1,2);
    states(0,0) = prev_occ_prob;
    states(0,1) = 1 - prev_occ_prob;

    for(unsigned i = 0; i < noOfProjections; i++){
        states = boost::numeric::ublas::prod(states, marchovM);
    }
    return states(0,0);
}

void Pmac_cell::init(double initialOccupancy, double initialFree)
{
    if(initialOccupancy > initialFree)
    {
        occupied_count = initialOccupancy;

        prev_occ_prob = 1;
        previous_is_occupied = true;
    }
    else
    {
        free_count = initialFree;

        prev_occ_prob = 0;
        previous_is_occupied = false;
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
     double lambda_entry = (entry + 1) / (free_count + 1); // a(1,2)
     double lambda_exit = (exit + 1) / (occupied_count + 1); // a(2,1)

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
         previous_is_occupied = (prev_occ_prob > 0.5) ? true : false;
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
