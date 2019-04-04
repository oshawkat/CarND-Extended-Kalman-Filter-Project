#define _USE_MATH_DEFINES

#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  cout << "FusionEKF initialization starting..." << endl;

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  Eigen::VectorXd x_ = VectorXd(4);
  Eigen::MatrixXd F_ = MatrixXd(4, 4);
  Eigen::MatrixXd P_ = MatrixXd(4, 4);
  Eigen::MatrixXd Q_ = MatrixXd(4, 4);
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Initial state transition matrix
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Initial state covariance matrix
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

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
    
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // Update timestamp
    previous_timestamp_ = measurement_pack.timestamp_ ;

    // Initialize state based on Radar measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Read in radar sensor measurements
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      // Ensure that the phi angle is within [-pi, pi]
      phi = std::remainder(phi, M_PI);

      // Calculate initial cartesian positions from radial Radar data
      float px, py;
      px = rho * cos(phi);
      py = rho * sin(phi);

      // Radar rho_dot is not a perfect measure of actual velocity but can
      // still provide some estimate
      float vx, vy;
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);

      ekf_.x_ << px, py, vx, vy;

    }
    // Initialize state based on Lidar measurement
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1], 
                 0, 0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  // dt - elapsed time expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */
  // Update state using EKF for Radar measurements (nonlinear)
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    // Because h(x') is non-linear, use the linear approximation instead to
    // preserve gaussians
    MatrixXd H_radar;
    Tools tool; // Useful for calcualting Jacobian and RMSE
    H_radar = tool.CalculateJacobian(ekf_.x_);
    ekf_.H_ = H_radar;
    
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  // Update state using standard KF for Lidar measurements 
  else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

}
