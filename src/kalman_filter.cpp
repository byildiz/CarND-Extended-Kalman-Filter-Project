#include <iostream>
#include "tools.h"
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Q_in) {
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(float delta_t) {
  // pre-compute used terms to avoid re-calculation
  float dt2 = delta_t * delta_t / 2;

  // update the state transition matrix F according to the new elapsed time
  F_(0, 2) = delta_t;
  F_(1, 3) = delta_t;

  // update the process noise covariance matrix Q
  // use noise_ax = 9 and noise_ay = 9 for your Q matrix
  MatrixXd Qu(2, 2);
  Qu << 9, 0,
        0, 9;
  MatrixXd G(4, 2);
  G << dt2, 0,
       0, dt2,
       delta_t, 0,
       0, delta_t;
  MatrixXd Gt = G.transpose();
  Q_ = G * Qu * Gt;

  // predict state and state covariance
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = Tools::MeasurementFunction(x_);
  // if measurement and predicted bearing are nearby -pi and pi, there may be
  // jumping. To avoid jumping add or subtract one of them 2*pi
  if (fabs(z_pred(1) - z(1)) > M_PI) {
    if (z_pred(1) > z(1)) {
      z_pred(1) -= 2*M_PI;
    } else {
      z_pred(1) += 2*M_PI;
    }
  }
  VectorXd y = z - z_pred;
  UpdateState(y);
}

void KalmanFilter::UpdateState(const Eigen::VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = (P_ * Ht) * Si;

  // update state and state covariance
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}