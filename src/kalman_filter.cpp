#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
 }

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  
  VectorXd y = z - H_*x_;
  MatrixXd PHt = P_*(H_.transpose());
  MatrixXd S = H_*PHt + R_;
  MatrixXd K = PHt*(S.inverse());
  x_ = x_ + K*y;
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  VectorXd hx = VectorXd(3);
  float numerator = x_(0)*x_(2) + x_(1)*x_(3);
  float denominator = sqrt(pow(x_(0), 2) + pow(x_(1), 2));

  if((fabs(denominator)<0.00001)) {
    hx << 0, 0, 0;
  }else{
    hx << denominator, atan2(x_(1), x_(0)), numerator/denominator ;
  }
  
  VectorXd y = z - hx;

  // Normailze the y(1) to -pi pi
  while(fabs((fabs(y(1))- M_PI) > 0.00001)){
    float signX = copysignf(1.0, y(1));
    y(1) = 2*M_PI - signX*y(1);
  }

  MatrixXd PHt = P_*(H_.transpose());
  MatrixXd S = H_*PHt + R_;
  MatrixXd K = PHt*(S.inverse());
  x_ = x_ + K*y;
  P_ = (I_ - K * H_) * P_;
  
}
