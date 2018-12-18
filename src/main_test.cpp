#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "tools.h"
#include "measurement_package.h"
#include "FusionEKF.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestRMSE(const VectorXd &expected) {
  Tools tools;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // the input list of estimations
  VectorXd e(4);
  e << 1, 1, 0.2, 0.1;
  estimations.push_back(e);
  e << 2, 2, 0.3, 0.2;
  estimations.push_back(e);
  e << 3, 3, 0.4, 0.3;
  estimations.push_back(e);

  // the corresponding list of ground truth values
  VectorXd g(4);
  g << 1.1, 1.1, 0.3, 0.2;
  ground_truth.push_back(g);
  g << 2.1, 2.1, 0.4, 0.3;
  ground_truth.push_back(g);
  g << 3.1, 3.1, 0.5, 0.4;
  ground_truth.push_back(g);

  VectorXd calculated;
  vector<VectorXd>::iterator iter_es;
  vector<VectorXd>::iterator iter_gr;
  for (size_t i = 1; i <= estimations.size(); ++i) {
    iter_es = estimations.begin() + i;
    iter_gr = ground_truth.begin() + i;
    vector<VectorXd> sub_estimations(estimations.begin(), iter_es);
    vector<VectorXd> sub_ground_truth(ground_truth.begin(), iter_gr);
    calculated = tools.CalculateRMSE(sub_estimations, sub_ground_truth);
  }
  VectorXd diff = (expected - calculated).array().abs();
  if ((diff.array() < 0.001).all()) {
    cout << "passed" << endl;
  } else {
    cout << "expected:" << endl << expected << endl;
    cout << "actual:" << endl << calculated << endl;
    cout << "failed" << endl;
  }
  cout << endl;
}

void TestJacobian(const MatrixXd &expected) {
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd calculated = Tools::CalculateJacobian(x_predicted);
  MatrixXd diff = (expected - calculated).array().abs();
  if ((diff.array() < 0.001).all()) {
    cout << "passed" << endl;
  } else {
    cout << "expected:" << endl << expected << endl;
    cout << "actual:" << endl << calculated << endl;
    cout << "failed" << endl;
  }
  cout << endl;
}

void TestFusionEKF(const VectorXd &max_RMSE) {
  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  VectorXd RMSE;
  string sensor_measurement;
  while (getline(in_file, sensor_measurement)) {
    istringstream iss(sensor_measurement);
    MeasurementPackage meas_package = Tools::ParseMeasurement(iss);

    float px_gt, py_gt, vx_gt, vy_gt;
    iss >> px_gt >> py_gt >> vx_gt >> vy_gt;
    VectorXd gt_values(4);
    gt_values << px_gt, py_gt, vx_gt, vy_gt;
    ground_truth.push_back(gt_values);

    //Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);

    //Push the current estimation from the Kalman filter's state vector
    VectorXd estimate = fusionEKF.ekf_.x_;
    estimations.push_back(estimate);

    RMSE = tools.CalculateRMSE(estimations, ground_truth);
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  if (!(RMSE.array() > max_RMSE.array()).any()) {
    cout << "passed" << endl;
  } else {
    cout << "expected:" << endl << RMSE << endl;
    cout << "actual:" << endl << max_RMSE << endl;
    cout << "failed" << endl;
  }
  cout << endl;
}

int main() {
  // test RMSE
  VectorXd expected_rmse(4);
  expected_rmse << 0.1, 0.1, 0.1, 0.1;
  cout << "Testing RMSE: ";
  TestRMSE(expected_rmse);

  // test Jacobian
  MatrixXd expected_Hj(3, 4);
  expected_Hj << 0.447214, 0.894427, 0, 0,
                 -0.4, .2, 0, 0,
                 0, 0, 0.447214, 0.894427;
  cout << "Testing Jacobian: ";
  TestJacobian(expected_Hj);

  // test FusionEKF
  VectorXd max_RMSE(4);
  max_RMSE << .11, .11, 0.52, 0.52;
  cout << "Testing FusionEKF: ";
  TestFusionEKF(max_RMSE);

  return 0;
}