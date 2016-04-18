#ifndef PMAC_CELL_H
#define PMAC_CELL_H
#include <vector>
class Pmac_cell
{
public:
    Pmac_cell();
    void addProbability(double occ_prob, double timeStamp);
    double getLongTermOccupancyProb();
    double getProjectedOccupancyProbability(unsigned noOfProjections = 1);
    double getTemporalPrediction(int forward_steps);
    void init(double initialOccupancy, double initialFree);
    double observationSum();
    double getLastObservationTime();
    unsigned getMixingTime();
    double getLastTransitionTime();

    bool deserialize(const std::vector<double> &values);
    std::vector<double> serialize();

    int long_term_best;
    double getLambdaExit();
    double getLambdaEntry();
private:
    static constexpr int NO_INTIAL_UPDATES = 100;
    static constexpr int MAX_NO_OF_OBS = 500;
    static constexpr double FORGET_FACTOR = 500.0 / 501.0;
    double occupied_count, free_count;
    double entry, exit;
    double prev_occ_prob;
    bool previous_is_occupied;
    double lastObservedTime;
    double last_transition;
};

#endif // PMAC_CELL_H
