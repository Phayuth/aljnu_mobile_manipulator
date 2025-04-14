#include "aljnu_path/bezier.h"

Bezier::Bezier(){

}

void Bezier::compute(double t) {
    // control points
    double p0[2] = {0.0, 0.0};
    double p1[2] = {1.0, 1.0};
    double p2[2] = {2.0, 1.0};
    double p3[2] = {3.0, 0.0};

    double x = (1 - t) * ((1 - t) * ((1 - t) * p0[0] + t * p1[0]) + t * ((1 - t) * p1[0] + t * p2[0])) + t * ((1 - t) * ((1 - t) * p1[0] + t * p2[0]) + t * ((1 - t) * p2[0] + t * p3[0]));
    double y = (1 - t) * ((1 - t) * ((1 - t) * p0[1] + t * p1[1]) + t * ((1 - t) * p1[1] + t * p2[1])) + t * ((1 - t) * ((1 - t) * p1[1] + t * p2[1]) + t * ((1 - t) * p2[1] + t * p3[1]));
}