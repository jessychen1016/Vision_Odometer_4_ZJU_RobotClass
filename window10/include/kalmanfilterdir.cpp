#define _USE_MATH_DEFINES
#include "kalmanfilterdir.h"
#include <iostream>
#include <math.h>

KalmanFilterDir::KalmanFilterDir():
    x(0), A(1), q(0.004), H(1), r(0.01), p(10) {
}

double KalmanFilterDir::update(double z_measure) {
    double temp = z_measure > x ? -2 * M_PI : 2 * M_PI;

    while (std::abs(z_measure - x) > M_PI) z_measure += temp;

    x = A * x;
    p = A * A * p + q;

    /* Measurement */
    gain = p * H / (p * H * H + r);
    x = x + gain * (z_measure - H * x);
    p = (1 - gain * H) * p;
    return x;
}