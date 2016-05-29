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

    double getLambdaExit();
    double getLambdaEntry();
    static constexpr int STATIC_OCCUPIED_VALUE = -2;
private:
    static constexpr int NO_INTIAL_UPDATES = 100;
    static constexpr int MAX_NO_OF_OBS = 500;
    static constexpr double FORGET_FACTOR = 500.0 / 501.0;    
    static constexpr double LAMBDA_EXIT_FOR_STATIC_OCCUPIED = 0.15;
    static constexpr double OCC_FREE_RATION_FOR_STATIC_OCCUPIED = 2;
    double occupied_count, free_count;
    double entry, exit;
    double prev_occ_prob;
    double lastObservedTime;
    double last_transition;
    double last_exit_residue, last_entry_residue;
    unsigned last_exit_cnt, last_entry_cnt;
    double old_exit_residue, old_entry_residue;
    unsigned old_exit_cnt, old_entry_cnt;
    double estimated_occupancy_prob;


    int obsCnt;
};

#endif // PMAC_CELL_H
