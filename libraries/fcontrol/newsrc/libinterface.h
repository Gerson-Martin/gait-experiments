#ifndef LIBINTERFACE_H
#define LIBINTERFACE_H

#include <vector>
#include <valarray>
#include <iostream>

//if you #include <complex.h> before <fftw3.h>, then fftw_complex is defined to be the native complex type (std::complex)
//and you can manipulate it with ordinary arithmetic (e.g. x = y * (3+4*I) (http://www.fftw.org/fftw3_doc/Complex-numbers.html#Complex-numbers);
#include <complex>
#include <fftw3.h>

#define complex_std std::complex<double>

namespace li
{

//fftw lib
bool fft_params(const std::vector<double> &signal_sample, std::vector<complex_std > &spectral_sample);
bool fft(std::valarray<double> &signal, std::valarray<std::complex<double> > &spectral);
bool ifft(std::valarray<std::complex<double> > &spectral, std::valarray<double> &signal);


}// end namespace li

#endif // LIBINTERFACE_H
