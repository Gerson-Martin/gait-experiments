#include "PIDBlock.h"

using  namespace std;
long PIDBlock::Initial(double new_Ts)
{

    Ts=new_Ts;
        std::cout << "Ts : " <<Ts << std::endl;
    iBlock = SystemBlock(
                std::vector<double> {Ts/2,Ts/2},
                std::vector<double> {-1,1});
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1});

        // LPF implementation
    double N = 20;    // LPFfilter N
    dBlock = SystemBlock(
                std::vector<double> {-1*N,1*N},
                std::vector<double> {-1,1+N*Ts*1});
//                std::vector<double> {-1,  1},
//                std::vector<double> {0,      Ts*1});
    return 0;
    cp=0;
    ci=0;
    cd=0;

}

PIDBlock::PIDBlock()
{

    Initial(0.01);

    kp=1;
    ki=1;
    kd=1;

}

PIDBlock::PIDBlock(double new_kp, double new_ki, double new_kd, double new_Ts)
{
    Ts=new_Ts;
    Initial(new_Ts);

    kp=new_kp;
    ki=new_ki;
    kd=new_kd;


}

double PIDBlock::UpdateControl(double input)
{

    cp = input*kp;
    ci = ki*iBlock.OutputUpdate(input);
    cd = kd*dBlock.OutputUpdate(input);

//    cout << ci << "; ";
//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }


    state=cp+ci+cd;


    //apply saturation
    if(saturation == true)
    {
        if (state > maxOut)
        {
            state = maxOut;
            cerr << "Top saturation!! output: " << state << " ,maxOut" << maxOut << endl;
        }
        //apply saturation
        if (state < minOut)
        {
            state = minOut;
            cerr << "Bottom saturation!! output: " << state << " ,maxOut" << minOut << endl;
        }
    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "pid : " <<state << std::endl;

    return state;

}

double PIDBlock::UpdateControlILock(double input)
{

    cp = input*kp;
//    ci = ki*iBlock.OutputUpdate(input);
    cd = kd*dBlock.OutputUpdate(input);

//    cout << ci << "; ";
//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }


    state=cp+ci+cd;


    //apply saturation
    if(saturation == true)
    {
        if (state > maxOut)
        {
            state = maxOut;
            cerr << "Top saturation!! output: " << state << " ,maxOut" << maxOut << endl;
        }
        //apply saturation
        if (state < minOut)
        {
            state = minOut;
            cerr << "Bottom saturation!! output: " << state << " ,maxOut" << minOut << endl;
        }
    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "pid : " <<state << std::endl;

    return state;
}

double PIDBlock::OutputUpdate(double input)
{

    return UpdateControl(input);
}

long PIDBlock::AntiWindup(double minPlantSaturation, double maxPlantSaturation)
{
    iBlock.SetSaturation(minPlantSaturation, maxPlantSaturation);
    cout << "Using saturation limit on the integrator for Anti-Windup." << endl;
    return 0;
}

double PIDBlock::GetState() const
{
    return state;
}


//void PIDBlock::operator>>(double &output)
//{
//    output = state;

//}

//void PIDBlock::operator>>(SystemBlock &output)
//{
//    output.OutputUpdate(state);

//}
