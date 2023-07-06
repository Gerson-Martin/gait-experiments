#include "LIMP.hpp"

LIMP::LIMP(double _Ts, double _Zmodel, double _g)
{
    Ts=_Ts;
    zModel=_Zmodel;
    g=_g;

    calculateSystem();
}



void LIMP::setParam(double _Ts, double _Zmodel, double _g)
{
    Ts=_Ts;
    zModel=_Zmodel;
    g=_g;
    calculateSystem();
}


void LIMP::setInitialCondition(std::vector<double> _x0,double _tSingleSupport)
{
   tSingleSupport=_tSingleSupport;
   x0=_x0;
   SystemLIMP.InitialCondition(x0);
}


KDL::Trajectory_Segment LIMP::getTrayectory()
{
    double eqradius=1.0;
    KDL::Rotation rot;
    pathCom= new KDL::Path_Composite();

    std::vector<double> u0{0,0};
    for (double dt = 0; dt < tSingleSupport; dt+=Ts)
    {
        SystemLIMP.output(u0);
        std::vector<std::vector<double>> aux=SystemLIMP.getState();
        std::vector<double>x {aux[0][0],aux[1][0],aux[2][0],aux[3][0]};
        trajCOM.emplace_back(rot,KDL::Vector(x[0],x[2],zModel));
        pathCom->Add(new KDL::Path_Line(trajCOM[trajCOM.size()-2], trajCOM[trajCOM.size()-1], orient.Clone(), eqradius));
    }
    KDL::VelocityProfile_Rectangular profRect(0.1);
    double duration=pathCom->PathLength()/0.1;
    //pathCom->Write(std::cout);
    KDL::Trajectory_Segment trayectoria(pathCom, profRect.Clone(), duration);
    return trayectoria;
}

void LIMP::getTrayectory(std::vector<KDL::Frame> &steps,std::vector<std::vector<double> > &x,double lengthFoot,double widthFoot)
{
//    double eqradius=1.0;
//    KDL::Rotation rot;
//    pathCom = new KDL::Path_Composite();
//    std::vector<std::vector<double>> aux2=SystemLIMP.getState();
//    x0.insert(x0.begin(),{aux2[0][0],aux2[1][0],aux2[2][0],aux2[3][0]});
//    x.insert(x.end(),{x0[2]+steps[0].p.x(),x0[3],x0[0]+steps[0].p.y(),x0[1],zModel,0.0});
    std::vector<double> u0{0,0};
    double footHoldx;
    double footHoldy;
    double phaseWalking=0;
    for (int i = 0; i < steps.size(); i+=1)
    {
        footHoldx=steps[i].p.x();
        footHoldy=steps[i].p.y();
        for (double dt = 0; dt <= tSingleSupport; dt+=Ts)
        {
            SystemLIMP.output(u0);
            std::vector<std::vector<double>> aux=SystemLIMP.getState();
            x0.insert(x0.begin(),{aux[0][0],aux[1][0],aux[2][0],aux[3][0]});
            x.insert(x.end(),{x0[2]+footHoldx,x0[3],x0[0]+footHoldy,x0[1],zModel,0.0});
            double currentStep=x.size()-1;
            if( abs(x[currentStep][0]-footHoldx)<lengthFoot &&
                abs(x[currentStep][2]-footHoldy)<widthFoot/2){
                if(footHoldy<0)
                phaseWalking=-1;
                else
                    phaseWalking=1;
             }
            else{
                phaseWalking=0;
            }
            x[currentStep].push_back(phaseWalking);

            //x.insert(x.begin(),{aux[2][0],aux[3][0],aux[0][0],aux[1][0]});

//            trajCOM.emplace_back(rot,KDL::Vector(x[0]+footHoldx,x[2]+footHoldy,zModel));
//            pathCom->Add(new KDL::Path_Line(trajCOM[trajCOM.size()-2], trajCOM[trajCOM.size()-1], orient.Clone(), eqradius));
        }
        x0[0]*=-1;
        x0[2]*=-1;
        SystemLIMP.InitialCondition(x0);
    }

//    KDL::VelocityProfile_Rectangular profRect(0.1);
//    double duration=pathCom->PathLength()/0.1;
//    yInfo()<<"----------------"<<duration;
//    KDL::Trajectory_Segment trayectoria(pathCom, profRect.Clone(), duration);
//    return trayectoria;
}

KDL::Path_Composite* LIMP::getPath()
{
    return pathCom;
}

MatrixXd LIMP::discretizeMatrix(MatrixXd M, double Ts)
{
    MatrixXd W=M*Ts;

    // Taylor series for exp(A)
    MatrixXd E(W.cols(),W.cols());E.setZero( W.cols(),W.cols());
    MatrixXd F(W.cols(),W.cols());F.setZero( W.cols(),W.cols());

    for (int i=0;i<W.rows();i++) {
        F(i,i)=1.0;
    }


    double k = 1.0;

    while(norm1(F) > 0.0)
    {
       E = E + F;
       F = W*F/k;
       k = k+1.0;
    }
    return E;
}

double LIMP::norm1(MatrixXd W)
{
    double sumCol=0.0;
    double aboutSumCol=0.0;
    for (int i=0;i<W.cols();i++) {
        for (int j=0;j<W.rows();j++) {
            sumCol=W(j,i)+sumCol;
        }
        if(sumCol>aboutSumCol){
            aboutSumCol=sumCol;
        }
    }
    return aboutSumCol;
}

void LIMP::calculateSystem()
{
    MatrixXd A(4,4);
    A<< 0  , 1, 0,    0,
        g/zModel ,0, 0,    0,
        0   , 0 ,0   , 1,
        0 ,   0 ,g/zModel ,0;
    MatrixXd B(4,2);
    B<<    0.0 ,  0.0,
           1.0 ,  0.0,
           0.0 ,  0.0,
           0.0 ,  1.0;
    MatrixXd C(2,4);
    C<<1,0,0,0,
        0,0,1,0;
    MatrixXd D(2,2);
    D<<0.0,0.0,
        0.0,0.0;
    MatrixXd Ad=discretizeMatrix(A,Ts);

    SystemLIMP.setParam(Ad,B,C,D,Ts);
}

