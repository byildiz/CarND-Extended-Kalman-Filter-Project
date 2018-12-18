#ifndef TOOLS_H_
#define TOOLS_H_

#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Tools {
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static MatrixXd CalculateJacobian(const VectorXd &x_state);

  /**
   * A helper method for EKF measurement function.
   */
  static VectorXd MeasurementFunction(const VectorXd &x_state);

  /**
   * A helper method to parse sensor measurement string
   */
  static MeasurementPackage ParseMeasurement(istringstream &iss);

private:
  /**
   * Total residual to avoid re-summing each time
   */
  VectorXd total_residual_;
};

#endif /* TOOLS_H_ */
