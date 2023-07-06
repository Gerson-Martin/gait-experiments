#include "StateSpace.h"
#include <iostream>
///
/// \brief StateSpace::StateSpace: Apply an output saturation to block response. Only output saturation is
/// available. If you need an input saturation use the previous block output saturation.
/// \param low: Lowest value for the possible output.
/// \param high: Highest value for the possible output.
/// \return
///
StateSpace::StateSpace()
{
//    this->G.setZero(0,0);
//    this->H.setZero(0,0);
//    this->C.setZero(0,0);
//    this->D.setZero(0,0);
//    this->x.setZero(G.rows(),1);
//    this->y.setZero(C.rows(),1);
//    this->u.setZero(H.cols(),1);
//    this->dts=dts;
}
///
/// \brief StateSpace::StateSpace(MatrixXd _G,MatrixXd _H,MatrixXd _C,MatrixXd _D,double dts): System constructor in state space from its arrays.
/// Parameters expressed in Eigen's matrices.
/// \param _G: State Matrix.
/// \param _H: Input Matrix.
/// \param _C: Output Matrix.
/// \param _D: Feedthrough Matrix.
/// \return
///
StateSpace::StateSpace(MatrixXd _G,MatrixXd _H,MatrixXd _C,MatrixXd _D,double dts){
    this->G=_G;
    this->H=_H;
    this->C=_C;
    this->D=_D;
    this->x.setZero(G.rows(),1);
    this->y.setZero(C.rows(),1);
    this->u.setZero(H.cols(),1);
    this->dts=dts;
}

///
/// \brief StateSpace::StateSpace(std::vector<vector<double>> _G,std::vector<vector<double>> _H,std::vector<vector<double>>_C,std::vector<vector<double>>_D,double dts):
/// System constructor in state space from its arrays.
/// Parameters expressed in vector<vector<double>>.
/// \param _G: State Matrix.
/// \param _H: Input Matrix.
/// \param _C: Output Matrix.
/// \param _D: Feedthrough Matrix.
/// \return
///

StateSpace::StateSpace(std::vector<vector<double>> _G,std::vector<vector<double>> _H,std::vector<vector<double>>_C,std::vector<vector<double>>_D,double dts){
    this->n_state=_G.size();
    this->n_input=_H[0].size();
    this->n_output=_C.size();
    G=this->vector2matrix(_G);
    H=this->vector2matrix(_H);
    C=this->vector2matrix(_C);
    D=this->vector2matrix(_D);
    this->x.setZero(G.rows(),1);
    this->y.setZero(C.rows(),1);
    this->u.setZero(H.cols(),1);
    this->dts=dts;
}

void StateSpace::InitialCondition(vector<double> x0){
    x=vector2matrix(x0);

}

///
/// \brief StateSpace::vector2matrix(std::vector<vector<double> > V) It convert a vector to eigen's matrix.
/// \param V: Value of Vector class.
/// \return

MatrixXd StateSpace::vector2matrix(std::vector<vector<double> >V)
{
    int n_row=V.size();
    int n_col=V[0].size();
    MatrixXd M(n_row,n_col);

    for(int i=0;i<n_row;i++){
       for(int j=0;j<n_col;j++){
           M(i,j)=V[i][j];
       }
    }
    return M;
}
MatrixXd StateSpace::vector2matrix(vector<double>V)
{
    int n_row=V.size();
    int n_col=1;
    MatrixXd M(n_row,n_col);

    for(int i=0;i<n_row;i++){
       for(int j=0;j<n_col;j++){
           M(i,j)=V[i];
       }
    }
    return M;
}
///
/// \brief StateSpace::vector2matrix(std::vector<vector<double> >M) It convert a eigen's matrix to Vector.
/// \param M: Value of Eigen::Matrix class.
/// \return
///
std::vector<vector<double>> StateSpace::matrix2vector(MatrixXd M)
{
    std::vector<double> row;
    std::vector<vector<double>>v;
    for(int i=0;i<M.rows();i++){
       for(int j=0;j<M.cols();j++){
           row.push_back(M(i,j));
       }
       v.push_back(row);
       row.clear();
    }
    return v;
}

///
/// \brief StateSpace::output(double new_input): Get output's system as a double.
/// \param new_input: New value of input at system.
/// \return
///

double StateSpace::output(double new_input)
{
    u(0)=new_input;
    x=G*x+H*u;
    y=C*x+D*u;
    return y(0);
}

vector<vector<double>> StateSpace::output(vector<double> new_input)
{

    u=vector2matrix(new_input);
    y=C*x+D*u;
    x=G*x+H*u;
    return this->matrix2vector(y);
}
///
/// \brief StateSpace::getObservabilityMatrix(): Calculate observability Matrix of the system. Return a std::vector<vector<double>>.
/// \return
///

std::vector<vector<double>> StateSpace::getObservabilityMatrix()
{
    MatrixXd G_pow=this->G;
    MatrixXd aux;
    ObservabilityMatrix=this->C;
    for(int i=1;i<n_state;i++){
        aux=ObservabilityMatrix;
        ObservabilityMatrix.resize(this->C.rows()*(i+1),NoChange);
        ObservabilityMatrix<<aux,
                   this->C*G_pow;
        G_pow=G_pow*this->G;
    }
    return matrix2vector(ObservabilityMatrix);
}

///
/// \brief StateSpace::getControllabiltyMatrix(): Calculate controllabilty Matrix of the system. Return a std::vector<vector<double>>.
/// \return
///

std::vector<vector<double> > StateSpace::getControllabiltyMatrix()
{
    MatrixXd G_pow=this->G;
    MatrixXd aux;
    ControllabilityMatrix=this->H;
    for(int i=1;i<n_state;i++){
        aux=ControllabilityMatrix;
        ControllabilityMatrix.resize(NoChange,this->H.cols()*(i+1));
        ControllabilityMatrix<<aux,G_pow*H;
        G_pow=G_pow*this->G;
    }
    cout<<ControllabilityMatrix<<endl;
    return matrix2vector(ControllabilityMatrix);
}

///
/// \brief StateSpace::GetState(): Get state or current output system. Return a double.
/// \return
///

double StateSpace::GetState() const
{
    return y(0);
}

///
/// \brief StateSpace::getState(): Get state or output system as a vector<vector<double>>
/// \return
///

std::vector<vector<double>> StateSpace::getState()
{
    return this->matrix2vector(x);
}


///
/// \brief StateSpace::printSystem(): Print system information by console.
/// \return
///
void StateSpace::printSystem(){
    std::cout<<"System Information:"<<std::endl<<"Number of states:"<<n_state<<std::endl<<"Number of inputs: "<<n_input<<std::endl<<"Number of outputs: "<<n_output<<std::endl;
    std::cout<<"G:"<<std::endl<<G<<std::endl<<std::endl;
    std::cout<<"H:"<<std::endl<<H<<std::endl<<std::endl;
    std::cout<<"C:"<<std::endl<<C<<std::endl<<std::endl;
    std::cout<<"D:"<<std::endl<<D<<std::endl<<std::endl;

}

void StateSpace::setParam(MatrixXd _G, MatrixXd _H, MatrixXd _C, MatrixXd _D, double dts)
{
    this->G=_G;
     this->H=_H;
     this->C=_C;
     this->D=_D;
     this->x.setZero(G.rows(),1);
     this->y.setZero(C.rows(),1);
     this->u.setZero(H.cols(),1);
     this->dts=dts;

}
