#include "FPDBlock.h"

//FPDBlock::FPDBlock()
//{
//    FPDBlock(1, 1, 1, 0.001);
//}

FPDBlock::FPDBlock(double new_kp, double new_kd, double new_fex, double new_dts)
{
    kp=new_kp;
    kd=new_kd;
    fex=new_fex;
    dts= new_dts;
    s_e = FractionalController1DOF(fex,dts);
    maxOut=0;
    minOut=0;

}

double FPDBlock::OutputUpdate(double new_input)
{
//    double cp,ci,cd;
    input=new_input;

    state=input*kp + kd*s_e.OutputUpdate(input);

    //apply saturation
    if (maxOut!=0)
    {
        state = std::min(state,maxOut);
    }
    //apply saturation
    if (minOut!=0)
    {
        state = std::max(state,minOut);
    }

//    if (signbit(ci)!=signbit(input))
//    {
//        iBlock.Reset();
//        ci = ki*iBlock.OutputUpdate(input);
//        dBlock.Reset();
//        cd = kd*dBlock.OutputUpdate(input);
//    }


//    std::cout << "pid : " << cp << ","<< ci << ","<< cd << std::endl;
//    std::cout << "state : " <<state << std::endl;

    if (!isnormal(state)) return 0;
    return state;

}

long FPDBlock::ParameterUpdate(vector<double> new_params)
{
    kp=new_params[0];
    kd=new_params[1];
    if ( abs(fex-new_params[2])>0.05 )
    {

        fex=new_params[2];
        s_e_state = s_e.GetState();
        std::cout << "s_e_state : " <<s_e_state << std::endl;

        s_e = FractionalController1DOF(fex,dts);
        s_e.Reset(s_e_state);

    }

    return 0;
}

double FPDBlock::GetState() const
{
    return state;
}

long FPDBlock::PrintParameters()
{
        std::cout << "kp : " << kp << ", kd: "<< kd << ", fex: "<< fex << std::endl;
        return 0;
}

long FPDBlock::GetParameters(double &out_kp, double &out_kd, double &out_fex)
{
    out_kp = kp;
    out_kd = kd;
    out_fex = fex;
    return 0;

}

