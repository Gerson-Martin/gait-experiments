#include "BlockDiagram.h"

BlockDiagram::BlockDiagram()
{

}

BlockDiagram::BlockDiagram(std::vector<BaseBlock *> &new_chain)
{
    chain = new_chain;

}
