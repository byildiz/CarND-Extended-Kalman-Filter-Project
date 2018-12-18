#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
  /**
   * state vector
   */
  VectorXd x_;

  /**
   * state covariance matrix
   */
  MatrixXd P_;

  /**
   * state transition matrix
   */
  MatrixXd F_;

  /**
   * process covariance matrix
   */
  MatrixXd Q_;

  /**
   * measurement matrix
   */
  MatrixXd H_;

  /**
   * measurement covariance matrix
   */
  MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Initializes Kalman filter
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param Q_in Process covariance matrix
   */
  void Init(MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in);

  /**
   * Predicts the state and the state covariance
   * using the process model
   * @param delta_t Time between k and k+1 in s
   */
  void Predict(float delta_t);

  /**
   * Updates the state by using standard Kalman Filter equations. Measurement
   * matrix H and measurement covariance matrix must be set up before calling
   * this method.
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations. Measurement
   * matrix H (Jacobian for thhis method) and measurement covariance matrix
   * must be set up before calling this method.
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z);

  void SetMeasurementMatrices(const MatrixXd &R_in, const MatrixXd &H_in);

  void SetState(const VectorXd &x_in);

  VectorXd GetState();

  MatrixXd GetStateCovariance();

private:
  /**
   * Updates the state and covariance matrix with given error
   * After calculating error and measurement matrix H (Jacobian for EKF)
   * KF and EKF uses same calculation. This method is the common part of
   * measurement update.
   * @param y Error between measurement and prediction
   */
  void UpdateState(const VectorXd &y);
};

#endif /* KALMAN_FILTER_H_ */
