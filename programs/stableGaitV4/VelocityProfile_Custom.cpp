#include "VelocityProfile_Custom.h"
#include <kdl/utilities/error.h>
void KDL::VelocityProfile_Custom::SetProfile(double pos1, double pos2)
{
    return;
}

void KDL::VelocityProfile_Custom::SetProfileDuration(double pos1, double pos2, double duration)
{
    return;
}

void KDL::VelocityProfile_Custom::SetProfile(std::vector<double> _position)
{
    position=_position;
}

double KDL::VelocityProfile_Custom::Duration() const
{
    return dt*(position.size()-1);
}

double KDL::VelocityProfile_Custom::Pos(double time) const
{
    int i=0;
    if(time>Duration()){
        i=position.size()-1;
    }else i=time/dt;

    double vel=Vel(time);

    double pos=position[i]+vel*(time-dt*i);
    return pos;
}

double KDL::VelocityProfile_Custom::Vel(double time) const
{
    int i=0;
    if(time>Duration()){
        i=position.size()-1;
    }else i=time/dt;
    double vel=(position[i+1]-position[i])/dt;
    return vel;
}

double KDL::VelocityProfile_Custom::Acc(double time) const
{
    throw Error_MotionPlanning_Incompatible();
    return 0;
}

void KDL::VelocityProfile_Custom::Write(std::ostream &os) const
{
    for(int i;i<position.size()-1;i++){
        os<<Pos(i*dt)<<" "<<Vel(i*dt)<<" "<<Acc(i*dt)<<"\n";
    }
}

