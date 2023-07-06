/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <Eigen/Dense>
#include "StateSpace.h"
#pragma once

/**
* @brief The KalmanFilter class:
* This class its implements a filter based in Kalman.
*/

class KalmanFilter {

public:

  KalmanFilter(
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P,
      double dt
  );
  KalmanFilter();

  KalmanFilter(vector<vector<double>> A,vector<vector<double>> B,vector<vector<double>> C,
               vector<vector<double>> Q,vector<vector<double>> R,vector<vector<double>> P,double dts);

  void init();


  void init(double t0, vector<vector<double>> x0);
  void init(double t0, const Eigen::VectorXd& x0);


  void update(const Eigen::VectorXd& y);
  void update(std::vector<vector<double>> y);


  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);
  void update(std::vector<vector<double>> y, double dt, std::vector<vector<double>> A);

  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };
  MatrixXd vector2matrix(std::vector<vector<double>>,int n_row,int n_col);
  std::vector<vector<double>> matrix2vector(MatrixXd);

private:

  // Matrices for computation
  Eigen::MatrixXd A, B, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
  double u;
};
