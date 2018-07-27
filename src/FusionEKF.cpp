#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  _is_initialized = false;

  _previous_timestamp = 0;

  // initializing matrices
  _R_laser = MatrixXd(2, 2);
  _R_radar = MatrixXd(3, 3);
  _H_laser = MatrixXd(2, 4);

  // measurement covariance matrix - laser
  _R_laser << 0.0225, 0, 
              0, 0.0225;

  // measurement covariance matrix - radar
  _R_radar << 0.09, 0, 0, 
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix
  _H_laser << 1, 0, 0, 0,
              0, 1, 0, 0;

  _noise_ax = 9;
  _noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::_InitializeFilter(const MeasurementPackage &measurement_pack) {
  string sensor_name = measurement_pack.sensor_type_ == 0 ? "LASER" : "RADAR";

  cout << "Filter Initialization using "<<sensor_name<<" sensor:"<< endl;
  
  // px, py, vx, vy
  VectorXd x = VectorXd(4);

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar outputs polar coordinates
    float rho = measurement_pack.raw_measurements_[0];
    float theta = measurement_pack.raw_measurements_[1];
    // Converts from polar to cartesian coordinates
    x(0) = rho * cos(theta);  // px
    x(1) = rho * sin(theta);  // py
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser outputs raw px and py directly, not conversion necessary
    x(0) = measurement_pack.raw_measurements_(0);  // px
    x(1) = measurement_pack.raw_measurements_(1);  // py
  }

  // Note that although radar gives velocity data in the form of the range
  // rate, a radar measurement does not contain enough information to determine
  // the state variable velocities vx and vy.
  x(2) = 1; // vx
  x(3) = 1; // vy

  // Initial state covariance matrix
  MatrixXd P = MatrixXd::Identity(4, 4);

  // Gives more uncertainty to velocity (as we initially don't have enough
  // information about it)
  P(2, 2) = 1000;
  P(3, 3) = 1000;

  // Initial transition matrix, dt is 0
  MatrixXd F = MatrixXd::Identity(4, 4);

  // Process covariance matrix
  MatrixXd Q = MatrixXd::Zero(4, 4);

  // Filter initialization
  ekf_.Init(x, P, F, Q);
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!_is_initialized) {
    // First measurement, filter initialization
    _InitializeFilter(measurement_pack);
    // Records current timestamp
    _previous_timestamp = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    _is_initialized = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // delta time - expressed in seconds
  float dt = (measurement_pack.timestamp_ - _previous_timestamp) / 1000000.0;

  _previous_timestamp = measurement_pack.timestamp_;

  ekf_.Predict(dt, _noise_ax, _noise_ay);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    MatrixXd Hj = _tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj, _R_radar);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_, _H_laser, _R_laser);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
