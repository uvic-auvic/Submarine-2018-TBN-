#include "median_filter.hpp"

void median_filter::add_data(double new_data)
{       
    data.push_front(new_data);
    data.pop_back();
}

void median_filter::clear_data()
{       
    for(int i = 0; i < data.size(); i++)
    {   
        add_data(0.0);
    }
}

double median_filter::get_result()
{       
    int center = (data.size() - 1) / 2;
    return data[center];
}
