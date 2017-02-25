// MovingAverage.h
// Author: Ian Heidenberger
//
// MovingAverage calculates an n-deep sliding average.

/*
    To use in main code, construct an object with sufficient sample depth, i.e. for 100 samples do:
    MovingAverage accelMAV(100);
    
    Every time you read a new sensor value, do this to add it to the MAV:
    accelMAV<<VALUE;
    
    To test the moving average do something like this:
    if(accelMAV.getAvg()>5) { //LAUNCH DETECTED! }

*/
#include <queue>

class MovingAverage {
    public:
    MovingAverage(int size);
    ~MovingAverage();
    void operator<<(const float val);
    float getAvg();
    
    std::queue<float> elements;
    float avg;
    int nElements;
};
    