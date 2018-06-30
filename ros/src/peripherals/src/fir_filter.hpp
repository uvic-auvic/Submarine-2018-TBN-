#include <list>

class fir_filter{
public:
    fir_filter(std::list<double> filter_coefficients);
    fir_filter(double* filter_coefficients, uint8_t filter_length);
    void add_data(double new_data);
    double get_result();
private:
    std::list<double> filter_coefficients;
    std::list<double> data;
};
