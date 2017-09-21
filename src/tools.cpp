#include <iostream>

#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Local variables
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd residual;
  
  // Validate inputs
  if (estimations.size() == 0) {
    cout << "Error: The estimation vector size should not be zero" << endl;
  } else if (estimations.size() != ground_truth.size()) {
    cout << "Error: The estimation and ground truth vectors sizes are not equal"
         << endl;
  } else {
    //----------------
    // Calculate RMSE
    //----------------
    
    // Accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
      residual = estimations[i] - ground_truth[i];
      residual = residual.array() * residual.array();
      rmse += residual;
    }
    
    // Calculate the mean square root
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
  }
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  // Local variables
  MatrixXd Hj(3,4);
  float e = 0.0001;
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Sub-computation for denominator
  float denom = pow(px, 2) + pow(py, 2);
  
  // Validate input
  if (fabs(denom) < e) {
    cout << "Error: Division by Zero in the Hj matrix" << endl;
  } else {
    //-------------------------------
    // Compute the Jacobian matrix
    //-------------------------------
    
    // Sub-computations for the denominators, put here for efficiency
    float denom_root = sqrt(denom);
    float denom_pow_root = (pow(denom, 3.0 / 2.0));
    
    // Partial derivatives for - ρ
    Hj(0, 0) = px / denom_root;
    Hj(0, 1) = py / denom_root;
    Hj(0, 2) = 0;
    Hj(0, 3) = 0;
    
    // Partial derivatives for - φ
    Hj(1, 0) = -py / denom;
    Hj(1, 1) = px / denom;
    Hj(1, 2) = 0;
    Hj(1, 3) = 0;
    
    // Partial derivatives for - ρ'
    Hj(2, 0) = py * ((vx * py) - (vy * px)) / denom_pow_root;
    Hj(2, 1) = px * ((vy * px) - (vx * py)) / denom_pow_root;
    Hj(2, 2) = px / denom_root;
    Hj(2, 3) = py / denom_root;
  }
  
  return Hj;
}
