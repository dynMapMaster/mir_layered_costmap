#ifndef _ACTIVITYMAPCOMPONENT_HPP
#define _ACTIVITYMAPCOMPONENT_HPP

#include <utility>

enum activityState{
    freeState = 0,
    occupied = 1,
    unseen = 2
};

enum collapsingMethod{ meanDecisions = 1, medianDecisions = 2, rawCopy = 3};

class activityMapComponent
{
public:
    activityMapComponent();
    activityMapComponent(bool useforgetFactor, double forgetFactor);
    ~activityMapComponent();

    // Time in s
    void observedFree(double time);

    // Time in s
    void observedOccupied(double time);

    // Time in ms
    void observedFree(int time);

    // Time in ms
    void observedOccupied(int time);

    // Time in ms
    void observedFree(unsigned long time);

    // Time in ms
    void observedOccupied(unsigned long time);

    double getProbOfRelease();
    double getProbOfOccupy();

    void add(const activityMapComponent* newComponent, collapsingMethod method = rawCopy);

    std::pair<double, double> getLongTermProb();



    void printValues();

protected:
    double _releaseEvents, _occupyEvents, _noOfOccupiedObservations, _noOfFreeObservations;
    unsigned long _timestamp;
    activityState _lastObservedState;
    bool _useForgetFactor;
    double _forgetFactor;


    // Time in ms
    void _observedFree(unsigned long time);

    // Time in ms
    void _observedOccupied(unsigned long time);
};

#endif
