#include <math.h>
#include <stdio.h>

#include "Window.h"

double bessj0(double x);

/*******************************************************************************/

void WindowRect(double win[], const long nelm)
{
    int i;
    for (i = 0; i < nelm; i++)
        win[i] = 1.0;
}

void WindowHann(double win[], const long nelm)
{
    int i;
    for (i = 0; i < nelm; i++)
        win[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (double) nelm));
}

void WindowHamming(double win[], const long nelm)
{
    int i;
    for (i = 0; i < nelm; i++)
        win[i] = 0.53836 + 0.46164 * cos(2.0 * M_PI * i / (double) nelm);
}

void WindowBlackman(double win[], const long nelm, const double a)
{
    int i;
    for (i = 0; i < nelm; i++) {
        const double a0 = (1.0 - a) / 2.0, a1 = 0.5, a2 = a / 2.0;
        win[i] = a0 - a1 * cos(2.0 * M_PI * i / (double) nelm) +
          a2 * cos(4.0 * M_PI * i / (double) nelm);
    }
}

void WindowBlackmanHarris(double win[], const long nelm)
{
    int i;
    const double a0 = 0.35875, a1 = 0.48829, a2 = 0.14128, a3 = 0.01168;
    for (i = 0; i < nelm; i++)
        win[i] = a0 - a1 * cos(2.0 * M_PI * i / (double) nelm) +
          a2 * cos(4.0 * M_PI * i / (double) nelm) -
          a3 * cos(6.0 * M_PI * i / (double) nelm);
}

void WindowKaiserBessel(double win[], const long nelm, const double a)
{
    int i;
    for (i = 0; i < nelm; i++) {
        const double z = (i - (nelm / 2.0)) / (nelm / 2.0);
        win[i] = bessj0(M_PI * a * sqrt(1.0 - z * z)) / bessj0(M_PI * a);
    }
}

void WindowGauss(double win[], const long nelm, const double s)
{
    int i;
    for (i = 0; i < nelm; i++)
    {
        const double a = (nelm - 1.0) / 2.0;
        win[i] = exp(-0.5 * pow((i - a) / (s * a), 2.0));
    }
}
