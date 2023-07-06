#include "FPDTuner.h"

FPDTuner::FPDTuner(double new_pm, double new_wcg, double new_dts)
{
    pm=new_pm;
    wcg=new_wcg;
    dts=new_dts;

    //store the iso-m table in memory
    isom = TableInterpolation("lib/fcontrol/phi.m_exp.csv");
    params.resize(3);


}

long FPDTuner::TuneIsom(const SystemBlock &sys)
{


    ulong tslope,tphi;
    sys.GetMagnitudeAndPhase(dts,wcg,sysmag,sysphi);
    sysphi = sysphi*180/M_PI; //conversion to degrees
    sys.GetMagnitudeAndPhase(dts,wcg+dw,sysmag,slope); //reusing slope
    slope = slope*180/M_PI; //conversion to degrees
    slope = (slope-sysphi) / ( log10(wcg+dw)-log10(wcg) ); //degrees/decade
    tslope=ulong(abs(slope));

//    cout << "sysphi="<< sysphi << ", sysmag="<< sysmag << ", slope=" << slope << " (deg)" << endl;

//    cout << "System slope at wcg="<< wcg << ", m=" << slope << "deg/decade" << endl;


    phi=-180+pm-sysphi;
//        cout << "sysphi="<< sysphi << ", pm="<< pm << ", phi=" << phi << " (deg)" << endl;


    if (phi >0) //phi positive, derivative.
    {
//        cout << "Searching phi="<< ulong(phi) << ", m=" << tslope << "deg/decade" << endl;
        alpha=isom.GetTableValue(ulong(phi),tslope);
        tgp=tan(phi*M_PI/180);
        sahp = sin(alpha*M_PI/2); //sin of alpha half pi
        cahp = cos(alpha*M_PI/2); //cos of alpha half pi

    }
    else //phi negative, integrator, use derivative table changing phi sign.
    {
//        cout << "Searching phi="<< ulong(-phi) << ", m=" << tslope << "deg/decade" << endl;
        alpha=-isom.GetTableValue(ulong(-phi),tslope);
        tgp=tan(-phi*M_PI/180);
        sahp = sin(-alpha*M_PI/2); //sin of alpha half pi
        cahp = cos(-alpha*M_PI/2); //cos of alpha half pi

    }

    tau_x=1/(tgp/(sahp-tgp*cahp));
    tau_a=1/(tau_x*pow(wcg,alpha));

    //compute k as 1/abs(1+tau_a*jw^alpha)
    k=1/abs(1.0+tau_a*pow(wcg*complex<double>(0,1),alpha) );
    //now divide by plant amplitude
    k=k/sysmag;

    params[0]=k;
    params[1]=tau_a*k;
    params[2]=alpha;

    return 0;
}

long FPDTuner::TuneIsom(const SystemBlock &sys, vector<double> &out_params)
{
    TuneIsom(sys);
    out_params = params;
    return 0;
}



long FPDTuner::TuneIsom(const SystemBlock & sys, FPDBlock &con)
{

    TuneIsom(sys);

//    cout << "alpha: "<< alpha << ", tau_a: "<< tau_a << ", k: "<< k << endl;

    //then change the controller parameters
    con = FPDBlock(k,tau_a*k,alpha,dts);


    return 0;
}

long FPDTuner::TuneIsom(const vector<SystemBlock> &sys, FPDBlock &con)
{
    cout << "This function needs to be remodeled!" << endl;
    cout << "Compute combined systems by polynomial multiplication instead!" << endl;

    return -1;

    double alpha, tau_a, k;
    double tgp, tau_x, sahp, cahp;
    double slope = 0.0, dw=0.1;
    ulong tslope=0;
    long tphi=0;
    double tmag=1;

    for (ulong i=0; i<sys.size() ; i++)
    {
        sys[i].GetMagnitudeAndPhase(dts,wcg,sysmag,sysphi);
        sysphi = sysphi*180/M_PI; //conversion to degrees
        tphi+=long(sysphi);
        tmag*=sysmag;
        sys[i].GetMagnitudeAndPhase(dts,wcg+dw,sysmag,slope); //reusing slope
        slope = slope*180/M_PI; //conversion to degrees
        slope = (slope-sysphi) / ( log10(wcg+dw)-log10(wcg) ); //degrees/decade
        tslope+=ulong(abs(slope));
    }


//    cout << "sysphi="<< tphi << ", sysmag="<< tmag << ", slope=" << tslope << " (deg)" << endl;

//    cout << "System slope at wcg="<< wcg << ", m=" << tslope << "deg/decade" << endl;


    phi=-180+pm-tphi;
//        cout << "sysphi="<< sysphi<< ", m=" << tslope << ", phi=" << phi << " (deg)" << endl;


    if (phi >=0) //phi positive, derivative.
    {
//        cout << "Searching phi="<< ulong(phi) << ", m=" << tslope << "deg/decade" << endl;
        alpha=isom.GetTableValue(ulong(phi),tslope);
        tgp=tan(phi*M_PI/180);
        sahp = sin(alpha*M_PI/2); //sin of alpha half pi
        cahp = cos(alpha*M_PI/2); //cos of alpha half pi

    }
    else //phi negative, integrator, use derivative table changing phi sign.
    {
//        cout << "Searching phi="<< ulong(-phi) << ", m=" << tslope << "deg/decade" << endl;
        alpha=-isom.GetTableValue(ulong(-phi),tslope);
        tgp=tan(-phi*M_PI/180);
        sahp = sin(-alpha*M_PI/2); //sin of alpha half pi
        cahp = cos(-alpha*M_PI/2); //cos of alpha half pi

    }

    tau_x=1/(tgp/(sahp-tgp*cahp));
    tau_a=1/(tau_x*pow(wcg,alpha));

    //compute k as 1/abs(1+tau_a*jw^alpha)
    k=1/abs(1.0+tau_a*pow(wcg*complex<double>(0,1),alpha) );
    //now divide by plant amplitude
    k=k/tmag;



//    cout << "alpha: "<< alpha << ", tau_a: "<< tau_a << ", k: "<< k << endl;

    //then change the controller parameters
    con = FPDBlock(k,tau_a*k,alpha,dts);


    return 0;
}

long FPDTuner::GetParameters(double &out_kp, double &out_ka, double &out_alpha)
{
    out_kp=params[0];
    out_ka=params[1];
    out_alpha=params[3];
    return 0;
}



