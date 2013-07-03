#include <math.h>

/* modified Bessel function */

double bessj0(double X)
{
    const double P1 = 1.0;
    const double P2 = 3.5156229;
    const double P3 = 3.0899424;
    const double P4 = 1.2067429;
    const double P5 = 0.2659732;
    const double P6 = 0.360768e-1;
    const double P7 = 0.45813e-2;
    const double Q1 = 0.39894228;
    const double Q2 = 0.1328592e-1;
    const double Q3 = 0.225319e-2;
    const double Q4 = -0.157565e-2;
    const double Q5 = 0.916281e-2;
    const double Q6 = -0.2057706e-1;
    const double Q7 = 0.2635537e-1;
    const double Q8 = -0.1647633e-1;
    const double Q9 = 0.392377e-2;

    if (fabs(X) < 3.75) {
        double Y;
        Y = (X / 3.75) * (X / 3.75);
        return (P1 + Y * (P2 + Y *(P3 + Y * (P4 + Y * (P5 + Y * (P6 + Y * P7))))));
    }
    else {
        double AX, BX, Y;
        AX = fabs(X);
        Y = 3.75 / AX;
        BX = exp(AX) / sqrt(AX);
        AX = Q1 + Y * (Q2 + Y * (Q3 + Y * (Q4 + Y * (Q5 + Y * (Q6 + Y * (Q7 + Y * (Q8 + Y * Q9)))))));
        return (AX * BX);
    }
}
