#include "VelocityProfile_Custom.h"

void KDL::VelocityProfile_Custom::SetProfile(double pos1, double pos2)
{
    return;
}

void KDL::VelocityProfile_Custom::SetProfileDuration(double pos1, double pos2, double duration)
{
    return;
}

void KDL::VelocityProfile_Custom::SetProfile(std::vector<double> _profileVel)
{
    profileVel=_profileVel;
}

double KDL::VelocityProfile_Custom::Duration() const
{
    return dt*(profileVel.size()-1);
}

double KDL::VelocityProfile_Custom::Pos(double time) const
{
    int i=0;
    if(time>Duration()){
        i=profileVel.size()-1;
    }else i=time/dt;

    double vel_end=Vel(time);
    //integramos
    double integral=0;

    for(int n=0;n<i;n++){
        integral=+(profileVel[i]*dt);
    }
    double pos=integral+(vel_end*(time-dt*i));
    return pos;
}

double KDL::VelocityProfile_Custom::Vel(double time) const
{
    int i=0;
    if(time>Duration()){
        i=profileVel.size()-1;
    }else i=time/dt;
    double m=(profileVel[i+1]-profileVel[i])/dt;
    double pos=profileVel[i]+m*time;
    return pos;
}

double KDL::VelocityProfile_Custom::Acc(double time) const
{
    double aceleration;
    if(time>(Duration()-dt)){
        aceleration=(Vel(Duration())-Vel(Duration()-dt))/dt;
    }else aceleration=(Vel(time)-Vel(time+dt))/dt;
    return aceleration;
}

void KDL::VelocityProfile_Custom::Write(std::ostream &os) const
{
    for(int i;i<profileVel.size()-1;i++){
        os<<Pos(i*dt)<<" "<<Vel(i*dt)<<""<<Acc(i*dt)<<"\n";
    }
}

