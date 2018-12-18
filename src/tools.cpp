#include <iostream>
#include <sstream>
#include <cmath>
#include "tools.h"
#include "measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::Tools() {
  total_residual_ = VectorXd(4);
  total_residual_ << 0.0, 0.0, 0.0, 0.0;
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd RMSE(4);
  RMSE << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Sizes mismatch" << endl;
    return RMSE;
  }

  VectorXd residual;
  // accumulate the last squared residual
  int last_index = estimations.size() - 1;
  residual = estimations[last_index] - ground_truth[last_index];
  residual = residual.array() * residual.array();
  total_residual_ += residual;
  // for (size_t i = 0; i < estimations.size(); ++i) {
  //   residual = estimations[i] - ground_truth[i];
  //   residual = residual.array() * residual.array();
  //   RMSE += residual;
  // }

  //calculate the mean
  RMSE = total_residual_ / estimations.size();
  // RMSE = RMSE / estimations.size();

  //calculate the squared root
  RMSE = RMSE.array().sqrt();

  //return the result
  return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  // create matrix with appropriate dims
  MatrixXd Hj(3, 4);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

VectorXd Tools::MeasurementFunction(const VectorXd &x_state) {
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = sqrt(px*px + py*py);

  // create vector with appropriate dims
  VectorXd z(3);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "MeasurementFunction() - Error - Division by Zero" << endl;
    return z;
  }

  // compute the measurement
  float rho = c1;
  float phi = atan2(py, px);
  float rho_dot = (px*vx + py*vy) / c1;
  z << rho, phi, rho_dot;

  return z;
}

MeasurementPackage Tools::ParseMeasurement(istringstream &iss) {
  MeasurementPackage meas_package;

  // reads first element from the current line
  string sensor_type;
  iss >> sensor_type;

  if (sensor_type.compare("L") == 0) {
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float px, py;
    iss >> px >> py;
    meas_package.raw_measurements_ << px, py;
  } else if (sensor_type.compare("R") == 0) {
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float rho, phi, rho_dot;
    iss >> rho >> phi >> rho_dot;
    meas_package.raw_measurements_ << rho, phi, rho_dot;
  }

  long long timestamp;
  iss >> timestamp;
  meas_package.timestamp_ = timestamp;

  return meas_package;
}