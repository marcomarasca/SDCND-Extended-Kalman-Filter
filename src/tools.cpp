#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);

  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0) {
    cout << "Error - The estimations vector cannot be empty.";
    return rmse;
  }

  if (estimations.size() != ground_truth.size()) {
    cout << "Error - The estimations and ground truth vectors should be of the "
            "same size.";
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if (px == 0.0 && py == 0.0) {
    cout << "[Error]: CalculateJacobian - Division by Zero" << endl;
    return Hj;
  }

  float px_2_py_2_sum = px * px + py * py;

  // Check division by zero
  if (fabs(px_2_py_2_sum) < 0.0001) {
    cout << "[Error]: CalculateJacobian - Division by Zero" << endl;
    return Hj;
  }

  float px_2_py_2_sum_sq = sqrt(px_2_py_2_sum);
  float px_2_py_2_sum_pow = pow(px_2_py_2_sum, 3.0 / 2.0);

  // Computes the Jacobian matrix
  Hj << px / px_2_py_2_sum_sq, py / px_2_py_2_sum_sq, 0, 0,
        -(py / px_2_py_2_sum), px / px_2_py_2_sum, 0, 0,
        py * (vx * py - vy * px) / px_2_py_2_sum_pow, px * (vy * px - vx * py) / px_2_py_2_sum_pow, px / px_2_py_2_sum_sq, py / px_2_py_2_sum_sq;

  return Hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd &x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = sqrt(px * px + py * py);
  float theta = atan2(py, px);
  float rho_dot = 0.0;

  if (fabs(rho) >= 0.0001) {
    rho_dot = (px * vx + py * vy) / rho;
  }

  VectorXd z = VectorXd(3);
  z << rho, theta, rho_dot;

  return z;
}

float Tools::NormalizeAngle(float angle) {
  return atan2(sin(angle), cos(angle));
}
