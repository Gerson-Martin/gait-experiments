#include "ControllerBlock.h"

ControllerBlock::ControllerBlock() : SystemBlock()//default transfer is G=1/1
{
    TimeSignal default_signal; //default signal is 256 data, all zeros

    //TransferFunction default_transfer;
    InitControllerBlock(default_signal);


}

ControllerBlock::ControllerBlock(TransferFunction new_transfer) : SystemBlock(new_transfer)
{
    TimeSignal default_signal; //default signal is 256 data, all zeros
    //TransferFunction default_transfer; //default transfer is G=1/1

    InitControllerBlock(default_signal);
}

ControllerBlock::ControllerBlock(TransferFunction new_transfer, TimeSignal new_signal) : SystemBlock(new_transfer)
{
    InitControllerBlock(new_signal);


}

double ControllerBlock::ControlSignal(double new_error)
{

//    double controlValue = TimeResponseUpdate(errorSignal, new_error);
//    errorSignal.data.erase(errorSignal.data.begin());
//    errorSignal.data.push_back(new_error);
//    return controlValue;
    return 0;

}

bool ControllerBlock::InitControllerBlock(TimeSignal init_signal)
{
    errorSignal = init_signal;


    return true;
}



