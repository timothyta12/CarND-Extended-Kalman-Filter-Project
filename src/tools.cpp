#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  
  // Checking validity of input
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
  	return rmse;
  }
      
  // Accumulate
  for (int i=0; i<estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  
  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate square root
  rmse = rmse.array().sqrt();
      
  return rmse;
}
  
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  
  // Check for division by zero
  
}
