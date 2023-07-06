#include "FractionalController1DOF.h"

FractionalController1DOF::FractionalController1DOF()
{

}

FractionalController1DOF::FractionalController1DOF(double new_exp, double new_dts)
{
    dts=new_dts;
    if (new_exp < 0)
    {
//        cout << "Using fractional derivative for exponents lesser than one. Splitting." << endl;
        exp=1+modf(new_exp,&iexp);
        iexp--; //add one integrator
        iexp=-iexp;//iexp is only used for integral exponents
//        cout << "Integer exponent: " << iexp << " fractional exponent: " << exp << endl;
        //initialize the integer part
//        cout << iexp;
        ipart.resize(ulong(iexp));
        ipart[0] = SystemBlock(
//                    std::vector<double> {dts/2,dts/2},
//                    std::vector<double> {-1,1});
                    std::vector<double> {0,dts*1},
                    std::vector<double> {-1,1});

        for  (ulong i=1; i<iexp; i++)
        {
            ipart[i] = SystemBlock(
    //                    std::vector<double> {dts/2,dts/2},
    //                    std::vector<double> {-1,1});
                        std::vector<double> {0,1},
                        std::vector<double> {-1,1});
        }
        Init(exp,1);
        return;
    }
 /*   if (new_exp > 1)
    {
        cout << "Using fractional derivative for exponents greater than one. Splitting." << endl;
        exp=modf(new_exp,&iexp);
        cout << "Integer exponent: " << iexp << " fractional exponent: " << exp << endl;
        //initialize the integer part

        // LPF implementation
        double N = 20;    // LPFfilter N
        ipart = SystemBlock(
                    std::vector<double> {-1*N,1*N},
                    std::vector<double> {-1,1+N*dts*1});
        //                std::vector<double> {-1,  1},
        //                std::vector<double> {0,      Ts*1});
        exp =new_exp;
        Init(exp,new_dts);
        return;
    }
*/
    //otherwise
    exp = new_exp;
    iexp=0; //no integrators

//    cout << "Integer exponent: " << 0 << " fractional exponent: " << exp << endl;
//    ipart = SystemBlock(
//                std::vector<double> {1},
//                std::vector<double> {1});
    Init(exp,new_dts);
    return;


}

double FractionalController1DOF::OutputUpdate(double new_input)
{

    if (iexp==0)
    {
        //derivative operator
        state = new_input > fpart;
//        cout << "new_input " << new_input << " state (f) " << state << endl;
    }
    else
    {
        //integral operator
        //integrate first term
        state = new_input > ipart[0];
        //now integrate till iexp
        for (int i = 1; i < iexp; ++i)
        {
            state = state > ipart[i];
        }
        //finally fit the fractional exponent with fractional derivative
//        cout << "new_input " << new_input << " state (i) " << state << endl;

        state = state > fpart;
//        cout << "new_input " << new_input << " state (f) " << state << endl;

    }


    return state;

}

long FractionalController1DOF::ParameterUpdate(double new_exp, double new_dts)
{

    return 0;
}

long FractionalController1DOF::Reset(double new_state)
{
    state = new_state;
    return 0;

}
/*
FractionalController1DOF::FractionalController1DOF(double new_exp, double new_dts)
{
    dts=new_dts;

    exp=min(new_exp,1.99);
    exp=max(exp,-1.99);

    Init(exp,new_dts);

    cout << "Integer exponent: " << 0 << " fractional exponent: " << exp << endl;

    return;


}

double FractionalController1DOF::OutputUpdate(double new_input)
{
    state = new_input > fpart;
    cout << "new_input " << new_input << " state " << state << endl;

    return state;

}
*/
long FractionalController1DOF::Init(double new_exp, double new_dts)
{

    dts=new_dts;
    exp=new_exp;
//    cout << "Integer exponent: " << iexp << " fractional derivative exponent: " << exp << endl;
    fpart = FractionalDerivative (exp, dts);


    return 0;
}

double FractionalController1DOF::GetState() const
{
    return state;
}


