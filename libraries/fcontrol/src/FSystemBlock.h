#ifndef FSYSTEMBLOCK_H
#define FSYSTEMBLOCK_H

#include <complex>
#define C_I std::complex<double>(0,1)
#define C_0 std::complex<double>(0,0)
//#include "libinterface.h"


#include "Block.h"
#include "TimeSignal.h"

class FSystemBlock : public BaseBlock
{
public:
    FSystemBlock();
    FSystemBlock(const TimeSignal &timeImpulseResponse);

    bool TimeResponse(TimeSignal &input, TimeSignal &output);
    double TimeResponseUpdate(const TimeSignal &old_input, const double &new_value);
    double OutputUpdate(double new_input);


    bool SignalParams(const TimeSignal &new_signalParams);
    double GetState();
    long Reset();
private:
    long FSystemBlockInit(const TimeSignal &init);

    std::valarray< std::complex<double> > IN,OUT; //spectral values of input and output.

    std::valarray<double> rI,iI;//frecuency values of input (real and imag).
    std::valarray<double> rO,iO;//frecuency values of output (real and imag).
    std::valarray< std::complex<double> > JW;//jw are imaginary numbers.
    std::valarray< std::complex<double> > G;//Processed Gain spectral values. Frequency block transference function.
    std::valarray<double> g;//Processed gain time values. Time block transference function (impulse response).
    std::valarray<double> convolution_n;

    //signal parameters
    double sFs,dts;
    unsigned int sN, jwN;
    double N,n;

    //state values
    double state;    
    std::valarray<double> oldStates;
    std::valarray<double> oldInputs;

    //saturation values
    double maxOut, minOut; // 0 values disables saturation
};

#endif // FSYSTEMBLOCK_H
