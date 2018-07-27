#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool _is_initialized;

  // previous timestamp
  long long _previous_timestamp;

  // tool object used to compute Jacobian and RMSE
  Tools _tools;

  // Laser measurement covariance matrix
  Eigen::MatrixXd _R_laser;
  // Radar measurement covariance matrix
  Eigen::MatrixXd _R_radar;
  // Laser measurement matrix
  Eigen::MatrixXd _H_laser;

  // Noise on the x axes
  float _noise_ax;
  // Noise on the y axes
  float _noise_ay;

  /**
   * Initilizes the kalman filter using the given measurement package, if the data comes from a radar
   * sensor converts the state coordinates from polar to cartesian.
   */ 
  void _InitializeFilter(const MeasurementPackage &measurement_pack);
};

#endif /* FusionEKF_H_ */
