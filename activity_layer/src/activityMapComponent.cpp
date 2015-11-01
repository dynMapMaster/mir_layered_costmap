#include "activityMapComponent.hpp"

#include <iostream>

using namespace std;


activityMapComponent::activityMapComponent()
{
    _lastObservedState = unseen;
    _noOfFreeObservations = 0;
    _noOfOccupiedObservations = 0;
    _occupyEvents = 0;
    _releaseEvents = 0;
    _timestamp = 0;


}

activityMapComponent::activityMapComponent(bool useforgetFactor, double forgetFactor, double max_val)
{
    _lastObservedState = unseen;
    _noOfFreeObservations = 0;
    _noOfOccupiedObservations = 0;
    _occupyEvents = 0;
    _releaseEvents = 0;
    _timestamp = 0;
    this->_useForgetFactor = useforgetFactor;
    this->_forgetFactor = forgetFactor;
    this->_maxValue = max_val;
}


activityMapComponent::~activityMapComponent(){}


void activityMapComponent::observedFree(double time)
{
    unsigned long timeMs = (unsigned long) (time * 1000);
    this->_observedFree(timeMs);
}

void activityMapComponent::observedFree(int time)
{
    unsigned long timeMs = (unsigned long) (time);
    this->_observedFree(timeMs);
}

void activityMapComponent::observedFree(unsigned long time)
{
    this->_observedFree(time);
}

void activityMapComponent::_observedFree(unsigned long time)
{
    if(_useForgetFactor)
    {
        _noOfFreeObservations++;
        if(_lastObservedState == occupied){
            _releaseEvents++;

        }
        _releaseEvents *= _maxValue/_noOfFreeObservations;
        _noOfFreeObservations = _maxValue;

        // Adjust inactive state
        _noOfOccupiedObservations = 1 + (_noOfOccupiedObservations-1) * _forgetFactor;
        _occupyEvents = 1 + (_occupyEvents-1) *_forgetFactor;
    }
    else
    {
        _noOfFreeObservations++;
        if(_lastObservedState == occupied){
            _releaseEvents++;
        }
        _lastObservedState = freeState;
    }
    _timestamp = time;
}

void activityMapComponent::setForgetting(bool on, double forgettingFactor, double maxValue)
{
    _forgetFactor = forgettingFactor;
    _useForgetFactor = on;
    _maxValue = maxValue;
}


void activityMapComponent::observedOccupied(double time)
{
    unsigned long timeMs = (unsigned long) (time * 1000);
    this->_observedOccupied(timeMs);
}

void activityMapComponent::observedOccupied(int time)
{
    unsigned long timeMs = (unsigned long) (time );
    this->_observedOccupied(timeMs);
}

void activityMapComponent::observedOccupied(unsigned long time)
{
    this->_observedOccupied(time);
}
void activityMapComponent::_observedOccupied(unsigned long time)
{
    if(_useForgetFactor)
    {
        _noOfOccupiedObservations++;
        if(_lastObservedState == freeState){
            _occupyEvents++;
        }
        _occupyEvents *= _maxValue/_noOfOccupiedObservations;
        _noOfOccupiedObservations = _maxValue;

        // Adjust inactive state
        _noOfFreeObservations = 1 + (_noOfFreeObservations-1) * _forgetFactor;
        _releaseEvents = 1 + (_releaseEvents-1) *_forgetFactor;
    }
    else
    {
        _noOfOccupiedObservations++;
        if(_lastObservedState == freeState){
            _occupyEvents++;
        }
        _lastObservedState = occupied;
    }
     _timestamp = time;
}

double activityMapComponent::getProbOfOccupy()
{
    //cout << _occupyEvents << "+1 / " << _noOfFreeObservations << " + 1" << endl;
    return (_occupyEvents+1) / ((double)_noOfFreeObservations+1);
}

double activityMapComponent::getProbOfRelease()
{
    //cout << _releaseEvents << "+1 / " << _noOfOccupiedObservations << " + 1" << endl;
    return (_releaseEvents+1) / ((double)_noOfOccupiedObservations+1);
}


void activityMapComponent::add(const activityMapComponent* newComponent, collapsingMethod method)
{
    switch (method) {
    case meanDecisions:
        throw "Not implemented - no difference from median";
        break;
    case medianDecisions:
        if(newComponent->_noOfFreeObservations >= newComponent->_noOfOccupiedObservations)
            this->observedFree(newComponent->_lastObservedState);
        else
            this->observedOccupied(newComponent->_lastObservedState);

        if(newComponent->_timestamp > this->_timestamp){
            this->_timestamp = newComponent->_timestamp;
            this->_lastObservedState = newComponent->_lastObservedState;
        }

        break;
    case rawCopy:
        this->_noOfFreeObservations += newComponent->_noOfFreeObservations;
        this->_noOfOccupiedObservations += newComponent->_noOfOccupiedObservations;
        this->_occupyEvents += newComponent->_occupyEvents;
        this->_releaseEvents += newComponent->_releaseEvents;

        if(newComponent->_timestamp > this->_timestamp){
            this->_timestamp = newComponent->_timestamp;
            this->_lastObservedState = newComponent->_lastObservedState;
        }
        break;
    default:
        throw "Add observation - unknown method";
        break;
    }
}

std::pair<double, double> activityMapComponent::getLongTermProb()
{
    double probOfFree = 0.5*(1-this->getProbOfOccupy())+0.5 * this->getProbOfRelease();
    double probOfOccupied = 0.5 * (1- this->getProbOfRelease()) + 0.5 * this->getProbOfOccupy();
    std::pair<double, double> result;
    result.first = probOfFree;
    result.second = probOfOccupied;
    return result;
}


void activityMapComponent::printValues()
{
    cout << "(" << this->_noOfFreeObservations << ", " << this->_noOfOccupiedObservations << ", " << this->_releaseEvents << ", " << this->_occupyEvents << ", " << this->_timestamp << ")";
}









