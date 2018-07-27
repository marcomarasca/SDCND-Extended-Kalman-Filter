#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() { _I = MatrixXd::Identity(4, 4); }

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(float dt, float noise_ax, float noise_ay) {
  // Updates the transition matrix F to take into account elapsed time
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Updates process noise covariance matrix
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  float dt_4_over4 = dt_4 / 4.0;
  float dt_3_over2 = dt_3 / 2.0;

  Q_ << dt_4_over4 * noise_ax, 0, dt_3_over2 * noise_ax, 0,
        0, dt_4_over4 * noise_ay, 0, dt_3_over2 * noise_ay, 
        dt_3_over2 * noise_ax, 0, dt_2 * noise_ax, 0,
        0, dt_3_over2 * noise_ay, 0, dt_2 * noise_ay;

  // Prediction
  x_ = F_ * x_;
  P_ = F_ * P_ * (F_.transpose()) + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H,
                          const MatrixXd &R) {
  VectorXd z_pred = H * x_;
  VectorXd y = z - z_pred;

  _UpdateMeasurement(y, H, R);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &H,
                             const MatrixXd &R) {
  VectorXd z_pred = _tools.CartesianToPolar(x_);
  VectorXd y = z - z_pred;

  // Normalize the angle between -PI, PI
  y(1) = _tools.NormalizeAngle(y(1));

  _UpdateMeasurement(y, H, R);
}

void KalmanFilter::_UpdateMeasurement(const VectorXd &y, const MatrixXd &H,
                                      const MatrixXd &R) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * (S.inverse());

  // New estimate
  x_ = x_ + (K * y);

  P_ = (_I - K * H) * P_;
}
