#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;  //state vector
  P_ = P_in;  //posteriori estimation covariance matrix
  F_ = F_in;  //state transition model
  H_ = H_in;  //observation model
  R_ = R_in;  //observation noise covariance
  Q_ = Q_in;  //process noise covariance
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F.transpose();
  P_ = F_ * P_ * Ft + Q_

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  I = MatrixXd::Identity(2, 2);
  P_ = (I - K * H_) * P_

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  VectorXd y = z - Hj * x_;
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  I = MatrixXd::Identity(2, 2);
  P_ = (I - K * H_) * P_
}
