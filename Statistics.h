#if !defined(Satistics_H)
#define Satistics_H

#include "common.h"

using namespace std;

class Statistics {
public:
    template <typename T>
    static void getMeanAndVariance(const vector<T> & _vec, double & _mean, double & _var);
    
    template <typename T>
    static void getMeanAndSTD(const vector<T> & _vec, double & _mean, double & _std);
    
    template <typename T>
    static void getMin(const vector<T> & _vec, double & _min);
    
    template <typename T>
    static void getMax(const vector<T> & _vec, double & _max);
    
    template <typename T>
    static void getMinAndMax(const vector<T> & _vec, double & _min, double & _max);
    
    template <typename T>
    static void getMedianWithCopyData(const vector<T> & _vec, double & _median);
    
    template <typename T>
    static void getMedianWithoutCopyData(vector<T> & _vec, double & _median);
    
    template <typename T>
    Statistics(const vector<T> & _vec);
    
    double mean, var, std, min, max;
};

#endif