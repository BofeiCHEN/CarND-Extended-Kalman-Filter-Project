#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  VectorXd x = VectorXd(4);
  MatrixXd P = MatrixXd(4, 4);
  MatrixXd F = MatrixXd(4, 4);
  MatrixXd Q = MatrixXd(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 100, 0,
       0, 0, 0, 100;
  F << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      ekf_.x_(0) = ro*cos(theta);
      ekf_.x_(1) = -ro*sin(theta);

    /* Review: 
    It is a good idea to use rho_dot for computing rough initial estimates for vx and vy!
    Try:
           float rho_dot = measurement_pack.raw_measurements_[2];
           ekf_.x_(2) =  rho_dot * cos(theta); //vx
           ekf_.x_(3) = rho_dot * sin(theta); //vy
    Note: ekf_.x_(1) = -ro*sin(theta); should be: ekf_.x_(1) = ro*sin(theta);
    */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
    }  

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // update F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update Q
  float dt_2 = pow(dt, 2);
  float dt_3 = pow(dt, 3);
  float dt_4 = pow(dt, 4);
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  
  ekf_.Q_ << (dt_4*noise_ax)/4.0, 0, (dt_3*noise_ax)/2.0, 0,
              0, (dt_4*noise_ay)/4.0, 0, (dt_3*noise_ay)/2.0,
              (dt_3*noise_ax)/2.0, 0, dt_2*noise_ax, 0,
              0, (dt_3*noise_ay)/2.0, 0, dt_2*noise_ay;


  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    VectorXd z = VectorXd(3);
    z << measurement_pack.raw_measurements_[0], 
         measurement_pack.raw_measurements_[1], 
         measurement_pack.raw_measurements_[2];

    // update Hj
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(z);

  } else {
    // TODO: Laser updates
    VectorXd z = VectorXd(2);
    z << measurement_pack.raw_measurements_[0], 
         measurement_pack.raw_measurements_[1];
    // update H
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
