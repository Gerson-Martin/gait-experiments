#ifndef SYSTEMBLOCKCHAIN_H
#define SYSTEMBLOCKCHAIN_H

#include <vector>
#include "SystemBlock.h"

class SystemBlockChain
{
public:
    SystemBlockChain();
    SystemBlockChain(const std::vector<SystemBlock> &new_chain);
    bool TimeResponse(double fs, const std::vector<double> &input, std::vector<double> &output);
private:
    std::vector<SystemBlock> chain;

};

#endif // SYSTEMBLOCKCHAIN_H
