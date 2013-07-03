#ifndef WINDOW_H
#define WINDOW_H

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

enum { RECT = 0, HANN, HAMMING, BLACKMAN, BLACKMANHARRIS, KAISERBESSEL, GAUSS, UNDEFINED };

/* static char *names[] = { "Rectangle", "Hann", "Hamming", "Blackman", "Blackman-Harris", "Kaiser-Bessel", "Gaussian" }; */

void WindowRect(double win[], const long nelm);
void WindowHann(double win[], const long nelm);
void WindowHamming(double win[], const long nelm);
void WindowBlackman(double win[], const long nelm, const double a);
void WindowBlackmanHarris(double win[], const long nelm);
void WindowKaiserBessel(double win[], const long nelm, const double a);
void WindowGauss(double win[], const long nelm, const double s);

#endif /* WINDOW_H */
