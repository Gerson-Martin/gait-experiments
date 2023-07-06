/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "KalmanFilter.hpp"

///
/// \brief KalmanFilter::KalmanFilter(double dt,const Eigen::MatrixXd& A,const Eigen::MatrixXd& B,const Eigen::MatrixXd& C,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,const Eigen::MatrixXd& P)):
/// Create a Kalman filter with the specified matrices as Eigen matrices.
///  \param A: System dynamics matrix
///  \param C: Output matrix
///  \param Q: Process noise covariance
///  \param R: Measurement noise covariance
///  \param P: Estimate error covariance
///  \return
///

 KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    double dt
    )
  : A(A), B(B),C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
  if(C.cols()!=n || B.rows()!=n)
      throw std::runtime_error("Filter's matrix: Range do not match");
}

///
/// \brief KalmanFilter::KalmanFilter(double dt,const Eigen::MatrixXd& A,const Eigen::MatrixXd& B,const Eigen::MatrixXd& C,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,const Eigen::MatrixXd& P)):
/// Create a Kalman filter with the specified his matrices as vector<vector<double> > .
///  \param A: System dynamics matrix
///  \param C: Output matrix
///  \param Q: Process noise covariance
///  \param R: Measurement noise covariance
///  \param P: Estimate error covariance
///  \return
///


KalmanFilter::KalmanFilter(vector<vector<double> > _A, vector<vector<double> > _B, vector<vector<double> > _C, vector<vector<double> > _Q, vector<vector<double> > _R, vector<vector<double> > _P, double dts)
{
    A=vector2matrix(_A,_A.size(),_A[0].size());
    B=vector2matrix(_B,_B.size(),_B[0].size());
    C=vector2matrix(_C,_C.size(),_C[0].size());
    Q=vector2matrix(_Q,_Q.size(),_Q[0].size());
    R=vector2matrix(_R,_R.size(),_R[0].size());
    P0=vector2matrix(_P,_P.size(),_P[0].size());
    this->dt=dts;
    m=C.rows();
    n=A.rows();
    initialized=false;
    I.setZero(n, n);
    I.setIdentity();
    x_hat.setZero(n);
    x_hat_new.setZero(n);
    if(C.cols()!=n || B.rows()!=n)
        throw std::runtime_error("Filter's matrix: Range do not match");
    u=0;
}
/// \brief KalmanFilter::KalmanFilter(): Create a blank estimator.
/// \return
KalmanFilter::KalmanFilter() {}

MatrixXd KalmanFilter::vector2matrix(std::vector<vector<double>> _M, int n_row, int n_col)
{
    MatrixXd M(n_row,n_col);

    for(int i=0;i<n_row;i++){
       for(int j=0;j<n_col;j++){
           M(i,j)=_M[i][j];
       }
    }
    return M;
}

std::vector<vector<double>> KalmanFilter::matrix2vector(MatrixXd M)
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

/// \brief KalmanFilter::init(double t0, const Eigen::VectorXd& x0):
/// Initialize the filter with a guess for initial states(Eigen::VectorXd).
/// \param t0: initial time.
/// \param x0: initial states.
/// \return

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

/// \brief KalmanFilter::init(double t0, const Eigen::VectorXd& x0):
/// Initialize the filter with initial states as zero.
/// \param t0: initial time.
/// \param x0: initial state.
/// \return
///

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

/// \brief KalmanFilter::init(double t0, const Eigen::VectorXd& x0):
/// Initialize the filter with initial states(vector<vector<double>>).
/// \param t0: initial time.
/// \param x0: initial state.
/// \return
///

void KalmanFilter::init(double t0, vector<vector<double>> x0)
{
    x_hat = vector2matrix(x0,x0.size(),x0[0].size());
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
}

/// \brief KalmanFilter::update(const Eigen::VectorXd& y):
/// Update the estimated state based on measured values(Eigen::VectorXd). The
/// time step is assumed to remain constant.
/// \param y: Measurements at the output of the system.
/// \return
///

void KalmanFilter::update(const Eigen::VectorXd& y) {
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  x_hat_new = A * x_hat+B*u;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;
  t += dt;
}

/// \brief KalmanFilter::update(const std::vector<vector<double>>y):
/// Update the estimated state based on measured values(std::vector<vector<double>>). The
/// time step is assumed to remain constant.
/// \param _y: Measurements at the output of the system.
/// \return
///

void KalmanFilter::update(std::vector<vector<double>> _y)
{
    VectorXd y= vector2matrix(_y,_y.size(),_y[0].size());
    if(!initialized)
      throw std::runtime_error("Filter is not initialized!");
    x_hat_new = A * x_hat+B*u;
    P = A*P*A.transpose() + Q;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    x_hat_new += K * (y - C*x_hat_new);
    P = (I - K*C)*P;
    x_hat = x_hat_new;
    t += dt;
}

/// \brief KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A):
/// Update the estimated state based on measured values,
/// using the given time step and dynamics matrix.
/// \param y: Measurements at the output of the system. Parameters expressed in Eigen matrix.
/// \param dt: new dt.
/// \param A: new state matrices.
/// \return
///
void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {
  this->A = A;
  this->dt = dt;
  update(y);
}

/// \brief KalmanFilter::update(std::vector<vector<double>> y, double dt, std::vector<vector<double> > A):
/// Update the estimated state based on measured values, using the given time step and dynamics matrix.
/// Parameters expressed in vector<vector<>>.
/// \param y: Measurements at the output of the system.
/// \param dt: new dt.
/// \param A: new state matrices.
/// \return
///

void KalmanFilter::update(std::vector<vector<double>> y, double dt, std::vector<vector<double> > A)
{
    this->A = vector2matrix(A,A.size(),A[0].size());
    this->dt = dt;
    update(y);
}
