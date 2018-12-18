#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include "kalman_filter.h"

using Eigen::MatrixXd;

class FusionEKF {
public:
  /**
   * Constructor
   */
  FusionEKF();

  /**
   * Destructor
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
  /**
   * Initialization flag. True if inialization after first measurement
   */
  bool is_initialized_;

  /**
   * Previous measurement timestamp
   */
  long long previous_timestamp_;

  /**
   * Measurement covariance matrix for laser
   */
  MatrixXd R_laser_;

  /**
   * Measurement covariance matrix for radar
   */
  MatrixXd R_radar_;

  /**
   * Measurement matrix for laser
   */
  MatrixXd H_laser_;
};

#endif /* FusionEKF_H_ */
