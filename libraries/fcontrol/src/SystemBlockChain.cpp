#include "SystemBlockChain.h"

SystemBlockChain::SystemBlockChain()
{

}

SystemBlockChain::SystemBlockChain(const std::vector<SystemBlock> &new_chain)
{
    chain = new_chain;

}

bool SystemBlockChain::TimeResponse(double fs, const std::vector<double> &input, std::vector<double> &output)
{
    std::vector<double> I;
    //get the fft from input
    I=input;//todo fft
    //compute for every block in the chain
    for (int i=0; i<chain.size(); i++)
    {

    }
    return 0;

}
