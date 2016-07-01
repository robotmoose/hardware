#ifndef GCCPHAT
#define GCCPHAT

#include <complex.h>
#include <fftw3.h>
#include <utility> // for std::pair
#include <cstdio> // for std::printf & std::sprintf

void gccphatInit(int N);
void printGccphatResult(std::pair<double, int>, int, int, double);
std::pair<double, int> gccphat(fftw_complex *, fftw_complex *, fftw_complex *);//, int, fftw_plan &, fftw_plan &);

#endif
