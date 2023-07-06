#ifndef VELOCITYPROFILE_CUSTOM_H
#define VELOCITYPROFILE_CUSTOM_H

#include <kdl/velocityprofile.hpp>
#include <vector>
namespace KDL {


    /**
     * A Custom VelocityProfile. A constructor flag
     * indicates if the calculated profile should be starting
     * or ending.
     * @ingroup Motion
     */
class VelocityProfile_Custom: public VelocityProfile
{
   std::vector<double> profileVel;
   std::vector<double> position;
   double dt;
public:
   VelocityProfile_Custom(){}
    VelocityProfile_Custom(std::vector<double> _position,double _dt):
      position(_position) ,dt(_dt){}
//    virtual void SetProfile(double pos1,double pos2);
//    // sets a trajectory from pos1 to pos2 as fast as possible
    virtual void SetProfile(double pos1,double pos2);
    virtual void SetProfileDuration(
        double pos1,double pos2,double duration);
    // Sets a trajectory from pos1 to pos2 in <duration> seconds.
    // @post new.Duration() will not be shorter than the one obtained
    //       from SetProfile(pos1,pos2).

    virtual void SetProfile(std::vector<double> _position);
    virtual double Duration() const;
    virtual double Pos(double time) const;
    virtual double Vel(double time) const;
    virtual double Acc(double time) const;
    virtual void Write(std::ostream& os) const;
    virtual VelocityProfile* Clone() const{
        VelocityProfile_Custom* res =  new KDL::VelocityProfile_Custom(position,dt);
        return res;
    }
    virtual ~VelocityProfile_Custom() {}

 };
}
#endif // VELOCITYPROFILE_CUSTOM_H
