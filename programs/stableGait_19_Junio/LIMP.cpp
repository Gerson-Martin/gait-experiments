#include "LIMP.hpp"
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
    double startStep=2;
    double endStep=steps.size()-2;

    std::vector<double> u0{0,0};
    double footHoldx;
    double footHoldy;
    double phaseWalking=0;
    double point_initial_x=steps[0+startStep].p.x();
    double point_initial_y=-x0[0];
    for (int i = startStep; i <endStep; i+=1)
    {
        footHoldx=steps[i].p.x();
        footHoldy=steps[i].p.y();
//        std::cout<<footHoldx<<" "<<point_initial_x<<std::endl;
        steps[i].p[0]=point_initial_x;
        for (double dt = 0; dt <(tSingleSupport); dt+=Ts)
        {
            SystemLIMP.output(u0);
            std::vector<std::vector<double>> aux=SystemLIMP.getState();
            x0.insert(x0.begin(),{aux[0][0],aux[1][0],aux[2][0],aux[3][0]});
            x.insert(x.end(),{x0[2]+point_initial_x,x0[3],x0[0]+point_initial_y,x0[1],zModel,0.0});
            if( abs(x[x.size()-1][0]-footHoldx)<lengthFoot/2 &&
                abs(x[x.size()-1][2]-footHoldy)<widthFoot/2){
                if(footHoldy<0)
                phaseWalking=-1;
                else
                    phaseWalking=1;
             }
            else{
                phaseWalking=0;
            }
            x[x.size()-1].push_back(phaseWalking);

            //x.insert(x.begin(),{aux[2][0],aux[3][0],aux[0][0],aux[1][0]});

//            trajCOM.emplace_back(rot,KDL::Vector(x[0]+footHoldx,x[2]+footHoldy,zModel));
//            pathCom->Add(new KDL::Path_Line(trajCOM[trajCOM.size()-2], trajCOM[trajCOM.size()-1], orient.Clone(), eqradius));
        }

        x0[0]*=-1;
        x0[2]*=-1;
//        std::cout<<footHoldx<<" "<<x[x.size()-1][0]<<" "<<x0[2]<<std::endl;
        point_initial_x=x[x.size()-1][0]-x0[2];
        point_initial_y=-x0[0];
        SystemLIMP.InitialCondition(x0);
    }


}

void LIMP::getTrayectory(std::vector<KDL::Frame> &steps,std::vector<tuple<KDL::Frame,double>> &COM,double lengthFoot,double widthFoot)
{
    double startStep=2;
    double endStep=steps.size();

    std::vector<double> u0{0,0};
    double footHoldx;
    double footHoldy;
    double phaseWalking=0.0;
    double point_initial_x=steps[0+startStep].p.x();
    double point_initial_y=-x0[0];
    KDL::Vector x;
    for (int i = startStep; i <endStep; i+=1)
    {
        footHoldx=steps[i].p.x();
        footHoldy=steps[i].p.y();
        steps[i].p[0]=point_initial_x;
        for (double dt = 0; dt <(tSingleSupport); dt+=Ts)
        {
            SystemLIMP.output(u0);
            std::vector<std::vector<double>> aux=SystemLIMP.getState();
            x0.insert(x0.begin(),{aux[0][0],aux[1][0],aux[2][0],aux[3][0]});

            x={x0[2]+point_initial_x,x0[0]+point_initial_y,zModel};
            if( abs(x[0]-footHoldx)<lengthFoot/2 &&
                abs(x[1]-footHoldy)<widthFoot/2){
                if(footHoldy<0.0)
                phaseWalking=-1.0;
                else
                    phaseWalking=1.0;
             }
            else{
                phaseWalking=0.0;
            }
            COM.push_back(make_tuple(KDL::Frame(x),phaseWalking));

        }

        x0[0]*=-1;
        x0[2]*=-1;
        point_initial_x=x[0]-x0[2];
        point_initial_y=-x0[0];
        SystemLIMP.InitialCondition(x0);
    }



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
    //inv(A)*(expm(A*Ts)-expm(A*0))*B
    SystemLIMP.setParam(Ad,B,C,D,Ts);
}
