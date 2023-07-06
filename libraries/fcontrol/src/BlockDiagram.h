#ifndef BLOCKDIAGRAM_H
#define BLOCKDIAGRAM_H

#include "Block.h"

class BlockDiagram
{
public:
    BlockDiagram();
    BlockDiagram(std::vector<BaseBlock*> &new_chain);

private:
    std::vector<BaseBlock *> chain;
};

#endif // BLOCKDIAGRAM_H
