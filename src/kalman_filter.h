#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
 public:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &Q_in);

  /**
   * Predicts the state and the state covariance using the process model taking
   * into consideration the elapsed time and the noise
   * @param dt Time between k and k+1 in s
   * @param noise_ax Noise on the x axes
   * @param noise_ay Noise on the y axes
   */
  void Predict(float dt, float noise_ax, float noise_ay);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param H Measurement matrix
   * @param R Measurement covariance matrix
   */
  void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H,
              const Eigen::MatrixXd &R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   * @param H Measurement matrix
   * @param R Measurement covariance matrix
   */
  void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &H,
                 const Eigen::MatrixXd &R);

 private:
  // Identity matrix
  Eigen::MatrixXd _I;

  // Tool object used to work on polar coordinates
  Tools _tools;

  /**
   * Given the error estimate updates the state
   * @param y The error estimate
   */
  void _UpdateMeasurement(const Eigen::VectorXd &y, const Eigen::MatrixXd &H,
                          const Eigen::MatrixXd &R);
};

#endif /* KALMAN_FILTER_H_ */
