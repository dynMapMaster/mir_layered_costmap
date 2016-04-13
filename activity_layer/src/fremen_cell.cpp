#include "fremen_cell.h"
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
using namespace std;

Fremen_cell::Fremen_cell()
    : components(_NUM_PERIODICITIES), gain(0.5), prev_measurement(0.5),
      first_time(-1), last_time(-1), measurements(0), order(3), periods(_NUM_PERIODICITIES),
      freq_elements(order)
{
    for (int i = 0; i < periods.size(); ++i) {
        periods[i] = (_NUM_PERIODICITIES*3600)/(i+1);
    }
}

bool fremenSort(Fremen_cell::Freq_element e1, Fremen_cell::Freq_element e2)
{
    return e1.amplitude > e2.amplitude;
}

Fremen_cell::~Fremen_cell()
{

}

int Fremen_cell::update(unsigned char order)
{
    /*
    if(gain == 0.0 || measurements == 0)
    {
        //order = 0;
        return 0;
    }
    */
    if(!components.empty())
    {
        vector<Freq_element> freq(_NUM_PERIODICITIES);
        uint64_t duration = last_time - first_time;
        for (int i = 0; i < _NUM_PERIODICITIES; ++i) {
            double re = components[i].real_states - components[i].real_balance;
            double im = components[i].imag_states - components[i].imag_balance;
            if(periods[i] <= duration)
                freq[i].amplitude = sqrt(re*re+im*im)/measurements;
            else
                freq[i].amplitude = 0.0;
            if(freq[i].amplitude < _AMPLITUDE_THRESHOLD)
                freq[i].amplitude = 0.0;
            freq[i].phase = atan2(im,re);
            freq[i].period = periods[i];
        }
        sort(freq.begin(),freq.end(),fremenSort);
        freq_elements.resize(order);
        for (int i = 0; i < order; ++i) {
            freq_elements[i] = freq[i];
        }
    }
    else {
        freq_elements.resize(order);
        for(int i=0; i<order; i++) {
            freq_elements[i].amplitude = 0.0;
            freq_elements[i].phase = 0.0;
            freq_elements[i].period = periods[i];
        }
    }
    return 0;
}

double Fremen_cell::estimate(double time)
{
    update(order);
    double estimate = gain;
    for (int i = 0; i < order; ++i) {
        estimate += 2*freq_elements[i].amplitude*cos(time/freq_elements[i].period*2*M_PI-freq_elements[i].phase);
    }

    //estimate += (prev_measurement - estimate)*exp(-fabs(time-last_time) / 3600.0);

    if(estimate < 0.0 + _SATURATION)
        estimate = 0.0 + _SATURATION;
    else if(estimate > 1.0 - _SATURATION)
        estimate = 1.0 - _SATURATION;
/*
    if(estimate >= 0.4)
        estimate = 1.0;
*/
    return estimate;
}

void Fremen_cell::addProbability(double occ_prob, double time)
{    
    if(measurements == 0) first_time = time;
    last_time = time;
    prev_measurement = occ_prob;
    double old_gain = gain;
    double new_gain = occ_prob;

    gain = (gain*measurements+new_gain) / (measurements + 1);

    if(old_gain > 0) {
        for(int i=0; i<_NUM_PERIODICITIES; i++)
        {
            components[i].real_balance = gain*components[i].real_balance / old_gain;
            components[i].imag_balance = gain*components[i].imag_balance / old_gain;
        }
    }

    // recalculate spectral components
    for (int i = 0; i < _NUM_PERIODICITIES; ++i) {
        double angle = 2*M_PI*time/periods[i];
        components[i].real_states += occ_prob * cos(angle);
        components[i].imag_states += occ_prob * sin(angle);
        components[i].real_balance += gain * cos(angle);
        components[i].imag_balance += gain * sin(angle);
    }
    measurements++;
    //order = -1;
}

int Fremen_cell::evaluate_current(double time, double signal, unsigned char order, double error)
{

}
