#ifndef FREMEN_CELL_H
#define FREMEN_CELL_H
#include <vector>
#include <stdint.h>

extern double *periods;

class Fremen_cell
{

public:
    Fremen_cell();
    ~Fremen_cell();
    void addProbability(double occ_prob, double time);
    int evaluate_current(double time, double signal, unsigned char order, double error);
    double estimate(double time);

    std::vector<double> serialize();
    void deserialize(std::vector<double> &input);

    typedef struct {
        double amplitude;
        double phase;
        double period;
    } Freq_element;

    typedef struct {
        double real_states;
        double imag_states;
        double real_balance;
        double imag_balance;
    } Spectral_component;
private:
    constexpr static unsigned _NUM_PERIODICITIES = 20;
    constexpr static double _SATURATION = 0.0;//0.05; // minimum uncertainty when predicting
    constexpr static double _AMPLITUDE_THRESHOLD = 0.0;

    std::vector<double> periods;
    std::vector<Spectral_component> components;
    std::vector<Freq_element> freq_elements;
    unsigned char order;
    int measurements;
    double prev_measurement;
    int64_t first_time, last_time;
    double gain;
    void update();
    //bool fremenSort(Freq_element e1, Freq_element e2);
};

#endif // FREMEN_CELL_H
