#include "FactorSystemBlock.h"

FactorSystemBlock::FactorSystemBlock()
{
InitFactorSystemBlock(vector<double>{0},vector<double>{0},1);

}

FactorSystemBlock::FactorSystemBlock(const std::vector<double> &new_zeros, const std::vector<double> &new_poles, double new_gain)
{
    InitFactorSystemBlock(new_zeros, new_poles, new_gain);
}

double FactorSystemBlock::GetState() const
{
    return state;
}

long FactorSystemBlock::InitFactorSystemBlock(const std::vector<double> &new_zeros, const std::vector<double> &new_poles, double new_gain)
{

    poles = new_poles;
    zeros = new_zeros;

    if ( poles.size()<zeros.size() )
    {
        cout << "Warning!. Numerator higher order." << endl;
        order=zeros.size();
        for (int i=poles.size(); i<zeros.size(); i++)
        {
            poles.push_back(1);
        }
    }
    else
    {
        order=poles.size();
        order=poles.size();
        for (int i=zeros.size(); i<poles.size(); i++)
        {
            zeros.push_back(1);
        }
    }


    //compute tf as factors
    gs.clear();

    std::cout << " - > tf as factors. Order " << order <<  std::endl;

    for (int i=0; i<order; i++)
    {
        SystemBlock g(-zeros[i],1,-poles[i],1);
        gs.push_back(g);

    }

    gain = new_gain;

    std::cout << " < - tf as factors. gain: " << gain <<  std::endl;
//    std::cout << "]" << std::endl;

//    std::cout << "denCoef : [";
//    for (int i=0; i<new_denCoef.size();i++)
//    {
//        denCoef[i]=new_denCoef[i]/denGain;
//        std::cout << denCoef[i] << "," ;

//    }
//    std::cout << "]" << std::endl;


    maxOut = 0;
    minOut = 0;


    oldStates.clear();
    //size oldstates one less position to remove actual state (z^-0)
    for(int i=0;i<order-1;i++)
    {
        oldStates.push_back(0.0);
    }
    oldInputs.clear();
    for(int i=0;i<order;i++)
    {
        oldInputs.push_back(0.0);
    }
    state = 0;


return 0;
}


double FactorSystemBlock::OutputUpdate(double new_input)
{
    response=0;

    //check signal parameters


//    //apply only to updated value (new_value). As an update, older values are known.
//    //start from 1, due to old first value will drop from new.

    //int N=numCoef.size();
    oldInputs.erase(oldInputs.begin());
    //now add the last value
    oldInputs.push_back(new_input);

    //compute repsonse
    double in=new_input;
    double out=0;
    for (int i=0; i<order; i++)
    {
        out=gs[i].OutputUpdate(in);
        response +=out;
        in=out;
    }

    //apply gain
    response *= gain;
    //apply saturation
    if (maxOut!=0)
    {
        response = std::min(response,maxOut);
    }
    //apply saturation
    if (minOut!=0)
    {
        response = std::max(response,minOut);
    }
    //now add the last value after compute response
    oldStates.push_back(response);
    //delete first value
    oldStates.erase(oldStates.begin());

    state = response;

    return response;
}






