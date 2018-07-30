#ifndef _MEDIAN_FILTER_HPP_
#define _MEDIAN_FILTER_HPP_

#include <deque>
#include <cstdint>

#include "filter_base.hpp"

class median_filter : public filter_base
{       
public:
    median_filter(uint8_t size) : data(size) {}
    void add_data(double new_data);
    void clear_data();
    double get_result();
private:
    std::deque<double> data;
};

#endif
