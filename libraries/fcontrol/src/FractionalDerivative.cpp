#include "FractionalDerivative.h"

FractionalDerivative::FractionalDerivative()
{

    Init(0,1);
}


FractionalDerivative::FractionalDerivative(double new_exp, double new_dts)
{

    //no need to use this
//    if (new_exp > 1)
//    {
//        cout << "Cant use exponents greater than one. Use another option." << endl;
//        exp=modf(new_exp,&iexp);
//        cout << "Integer exponent" << iexp << " fractional exponent: " << exp << endl;
//        Init(0,new_dts);
//        return;
//    }
//    if (new_exp < -1)
//    {
//        cout << "Fractiona DERIVATIVE Can't use exponents lesser than -1. Use another option." << endl;
////        exp=modf(new_exp,&iexp);
////        cout << "Integer exponent" << iexp << " fractional exponent: " << exp << endl;
//        Init(0,new_dts);
//        return;
//    }
//        cout << "Fractional derivative exponent: " << new_exp << endl;

    Init(new_exp,new_dts);
    return;
}




long FractionalDerivative::Init(double new_exp, double new_dts)
{


    dts=new_dts;
    alfa=new_exp;

    if (alfa<0)
    {
        ialfa=floor(alfa);
        alfa-=ialfa;
    }
    else
    {
        ialfa=0;
    }



    double bi=0;

    firtol=0.001; //default. otherwise assign with setter
    vfir.clear();
//    vector<double> vfir;
//    cout << "Fractional derivative FIR: "<<endl;
//MacLaurin series expansions based approximation

    //first few values can not be computed using the trick below.
    for (int i=0; i<ceil(abs(alfa)); i++)
    {
        bi = tgamma(alfa+1) / (tgamma(i+1)*tgamma(alfa-i+1));
        if (isnan(bi)) bi=0;
        vfir.push_back( pow(-1,i)*bi/(pow(dts,alfa)) );
//        if(abs(vfir[i])<firtol) break;
//        cout << vfir[i] << ", ";
    }

    // this formula use the fir coefficients term (-1)^i to make lgamma sign alternate for negative values
    // lgamma(alfa+1-i), but only works for exponents lesser than one, therefore the ceil(abs(alfa))
    for (int i=ceil(abs(alfa)); i<FRACTIONALDERIVATIVE_MAXSIZE; i++)
    {
        //this option valid only for exponents greater than -1
//        bi = ialfa*pow(-1,ceil(abs(alfa)))*tgamma(alfa+1) / exp( lgamma(1+i) + lgamma(alfa+1-i));

        // Now using the integer integrator option for all negative exponents (see below)
        bi = pow(-1,ceil(abs(alfa)))*tgamma(alfa+1) / exp( lgamma(1+i) + lgamma(alfa+1-i));
        if (isnan(bi)) bi=0;
        vfir.push_back( bi/(pow(dts,alfa)) );
//        if(abs(vfir[i])<firtol) break;
//        cout << vfir[i] << ", ";
    }

//    cout << endl<< endl;

    N=vfir.size();

    fir.resize(N,0);
    oldStates.resize(N,0);
    oldInputs.resize(N,0);

    //
    //integrate the integer needed orders

    while (ialfa<0)
    {
        ialfa++;

        for (int i=0; i<N; i++)
        {
            fir[i]=0;
            for (int j=0; j<=i; j++)
            {
                fir[i]+=vfir[j];
            }
            fir[i]*=dts;
        }
        //and copy
        for (int i=0; i<N; i++)
        {
            vfir[i]=fir[i];

        }
    }

    //prepare fir for convolution (flip values)
    for (int i=0; i<N; i++)
    {
        fir[i]=vfir[N-i-1];
//        cout << fir[i] << ", ";
    }



    return 0;
}

vector<double> FractionalDerivative::GetVfir() const
{
    return vfir;
}


double FractionalDerivative::GetState() const
{
    return state;
}



double FractionalDerivative::OutputUpdate(double new_input)
{
    double response=0;
//    n++;
//    if (N<n)
//    {
//        n=0;
//        Reset();

//    }



    //TODO: Check complexity for cshift: maybe deque is better for that operation

    //prepare the array for multiplication
    oldInputs = oldInputs.shift(1); //move all data one position left
    oldInputs[oldInputs.size()-1]=new_input; //overwrite last value

    //compute response
    convolution_n = (oldInputs*fir);
    response = convolution_n.sum();

//    cout  << "[ ";
//    for (int i=0; i<oldInputs.size(); i++)
//    {
//        cout << oldInputs[i] << ", " ;
//    }
//    cout  << "] "<< endl;

    state = response;
    oldStates[0]=response;//overwrite first value
    oldStates = oldStates.cshift(1); //make the first value be the last one (make it i_n)

//    cout  << "[ ";
//    for (int i=0; i<oldStates.size(); i++)
//    {
//        cout << oldStates[i] << ", " ;
//    }
//    cout  << "] "<< endl << endl;
    //apply saturation
    if(saturation == true)
    {
        if (response > maxOut)
        {
            response = maxOut;
            cerr << "Top saturation!! output: " << response << " ,maxOut" << maxOut << endl;
        }
        //apply saturation
        if (response < minOut)
        {
            response = minOut;
            cerr << "Bottom saturation!! output: " << response << " ,maxOut" << minOut << endl;
        }
    }


    return response;
}
