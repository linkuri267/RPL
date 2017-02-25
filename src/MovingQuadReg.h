// MovingQuadReg.h
// Author: Ian Heidenberger
//
// MovingQuadReg calculates an n-deep sliding quadratic regression.

/* To use in main code, call addPoint every time we extract GPS altitude and pass the current
    microsecond Timer value DIVIDED BY 1 MILLION as well as the altitude in meters.
    Like so:
    addPoint(usTimerVal/1000000.0f, (float)altitude)
    
    During the apogee window of time, do this:
    
    gpsQuad.update();
    if(gpsQuad.getVelocity()<(-20) && gpsQuad.getRSQ()>0.8){ //FIRE PYRO! }
    
    I'll refine the threshold values based on some simulations.
    Probably need 5-10 packets or so, so construct the object as follows:
    MovingQuadReg gpsQuad(5);
*/
#include <deque>

class MovingQuadReg {
    public:
    MovingQuadReg(int size);
    ~MovingQuadReg();
    void addPoint(float t, float y);
    void update();
    float getVelocity();
    float getRSQ();
    float getA();
    float getB();
    float getC();
    
    private:
    std::deque<float> tvals, yvals;
    float A, B, C, tavg, yavg, RSQ;
    int nElements;
};
