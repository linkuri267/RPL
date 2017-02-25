// MovingQuadReg.cpp
// Author: Ian Heidenberger
//
// MovingQuadReg calculates an n-deep sliding quadratic regression.

#include "MovingQuadReg.h"

MovingQuadReg::MovingQuadReg(int size){
    nElements = size;
    A = 0.0f;
    B = 0.0f;
    C = 0.0f;
    for(int i=0;i<size;i++){ tvals.push_back(0); yvals.push_back(0); }
    tavg = 0.0f;
    yavg = 0.0f;
    RSQ = 0.0f;
}

MovingQuadReg::~MovingQuadReg(){
}

void MovingQuadReg::addPoint(float t, float y){
    tavg += (float)(t - tvals.front()) / nElements;
    yavg += (float)(y - yvals.front()) / nElements;
    tvals.pop_front();
    tvals.push_back(t);
    yvals.pop_front();
    yvals.push_back(y);
}

void MovingQuadReg::update(){
    //Calculate our sums matrix and the LSQ vector
    float a = 0.0f,b = 0.0f,c = 0.0f,d = 0.0f,e = 0.0f, z1 = 0.0f,z2 = 0.0f,z3 = 0.0f;
    float tt = 0.0f, yy = 0.0f,sq = 0.0f;
    for(int j=0;j<nElements;j++){
        tt = tvals[j];
        yy = yvals[j];
        sq = tt*tt;
        a += sq*sq;
        b += sq*tt;
        c += sq;
        d += tt;
        z1 += sq*yy;
        z2 += tt*yy;
    }
    e = nElements;
    z3 = yavg*nElements;
    //Invert the sums matrix
    float M11 = 0.0f,M12 = 0.0f,M13 = 0.0f,M22 = 0.0f,M23 = 0.0f,M33 = 0.0f,k = 0.0f;
    float tmp1 = (a*(c*e-d*d)-b*b*e+2*b*c*d-c*c*c);
    if (tmp1 == 0.0f) return;
    k = 1/tmp1;
    M11 = c*e - d*d;
    M12 = c*d - b*e;
    M13 = b*d - c*c;
    M22 = a*e - c*c;
    M23 = b*c - a*d;
    M33 = a*c - b*b;
    A = k*(M11*z1+M12*z2+M13*z3);
    B = k*(M12*z1+M22*z2+M23*z3);
    C = k*(M13*z1+M23*z2+M33*z3);
    float SSE = 0.0f, SST = 0.0f, t1 = 0.0f;
    for(int j=0;j<nElements;j++){
        tt = tvals[j];
        yy = yvals[j];
        t1 = yy - (A*tt*tt + B*tt + C);
        SSE += t1*t1;
        t1 = yy - yavg;
        SST += t1*t1;
    }
    if (SST == 0.0f) return;
    RSQ = 1 - (SSE/SST);
}

float MovingQuadReg::getVelocity(){
    return (2*A*tvals.back() + B);
}

float MovingQuadReg::getRSQ(){
    return RSQ;
}

float MovingQuadReg::getA(){
    return A;
}

float MovingQuadReg::getB(){
    return B;
}

float MovingQuadReg::getC(){
    return C;
}