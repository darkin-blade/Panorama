#if !defined(Rotation_H)
#define Rotation_H

#include "../common.h"

void angleConvert(const vector<vector<double> > rotations, vector<double> & new_rotations);
void sphere2Coordinate(const double sphere[], double coordinate[]);
void coordinate2Sphere(const double coordinate[], double sphere[]);
double sphereThetaConvert(const double pointM[], const double sphereP[]);
void rotateByVector(const double pointP[], const double vectorAxis[], const double _theta, double pointP_[]);

#endif