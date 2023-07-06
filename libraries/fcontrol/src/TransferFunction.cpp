#include "TransferFunction.h"

TransferFunction::TransferFunction()
{

    std::vector<double> nC = {1};
    std::vector<double> nE = {0};
    std::vector<double> dC = {1};
    std::vector<double> dE = {0};

    InitTransferFunction(nC,dC,nE,dE);

}

TransferFunction::TransferFunction(const std::vector<double> &new_num, const std::vector<double> &new_den)
{

    std::vector<double> nC,dC,nE,dE;

    nC.clear();
    nE.clear();

    dC.clear();
    dE.clear();


    for (int i=0; i<new_num.size(); i=i+2)
    {
        nC.push_back(new_num[i]);
        nE.push_back(new_num[i+1]);
    }

    for (int i=0; i<new_den.size(); i=i+2)
    {
        dC.push_back(new_den[i]);
        dE.push_back(new_den[i+1]);
    }


    InitTransferFunction(nC,dC,nE,dE);

}


TransferFunction::TransferFunction(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef,
                             const std::vector<double> &new_numExps, const std::vector<double> &new_denExps)
{
    InitTransferFunction(new_numCoef,new_denCoef,new_numExps,new_denExps);
}

std::vector<double> TransferFunction::getNumCoef() const
{
    return numCoef;
}

std::vector<double> TransferFunction::getDenCoef() const
{
    return denCoef;
}

std::vector<double> TransferFunction::getNumExps() const
{
    return numExps;
}

std::vector<double> TransferFunction::getDenExps() const
{
    return denExps;
}




bool TransferFunction::InitTransferFunction(const std::vector<double> &new_numCoef, const std::vector<double> &new_denCoef,
                                            const std::vector<double> &new_numExps, const std::vector<double> &new_denExps)
{
    if ( (new_numCoef.size()!=new_numExps.size())|(new_denCoef.size()!=new_denExps.size()) )
    {
        std::cout << "Wrong transfer function parameters" << std::endl;
        return false;
    }
    numCoef=new_numCoef;
    denCoef=new_denCoef;

    numExps=new_numExps;
    denExps=new_denExps;

    return true;

}
