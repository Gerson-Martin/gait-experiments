#include "Block.h"

BaseBlock::BaseBlock()
{
    saturation=false;
    maxOut = 0;
    minOut = 0;
}

///
/// \brief BaseBlock::SetSaturation: Apply an output saturation to block response. Only output saturation is
/// available. If you need an input saturation use the previous block output saturation.
/// \param low: Lowest value for the possible output.
/// \param high: Highest value for the possible output.
/// \return
///

long BaseBlock::SetSaturation(double low, double high)
{
    saturation=true;
    maxOut = high;
    minOut = low;

    return 0;
}



