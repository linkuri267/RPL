// MovingAverage.cpp
// Author: Ian Heidenberger
//
// MovingAverage creates an n-deep sliding average.

#include "MovingAverage.h"

MovingAverage::MovingAverage(int size){
    nElements = size;
    for(int i=0;i<size;i++){ elements.push(0); }
    avg = 0.0f;
}

MovingAverage::~MovingAverage(){
    std::queue<float> empty;
    std::swap( elements, empty );
}

void MovingAverage::operator<<(const float val){
    avg += (val - elements.front()) / nElements;
    elements.pop();
    elements.push(val);
}

float MovingAverage::getAvg(){
    return avg;
}