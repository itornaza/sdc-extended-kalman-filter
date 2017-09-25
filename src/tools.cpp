#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "tools.h"
#include "constants.h"

using namespace Constants;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

/**
 * Constructor
 */
Tools::Tools() {}

/**
 * Destructor
 */
Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
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
  
  // Return the RMSE
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // The Jacobian matrix
  MatrixXd Hj(3,4);
  
  // State parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Sub-computation for Hj elements
  float den = pow(px, 2) + pow(py, 2);
  float den_rt = sqrt(den);
  float den_p_rt = sqrt(pow(den, 3.0));
  float num_1 = py * ((vx * py) - (vy * px));
  float num_2 = px * ((vy * px) - (vx * py));
  
  // Validate input, ensure no divisions by zero
  if (fabs(den) < E1) {
    if (DEBUG) { cout << "Error: Division by Zero in the Hj matrix" << endl; }
    den = E1;
    den_rt = E1;
    den_p_rt = E1;
  }
  
  // Assign the elements to the Hj matrix
  Hj << px/den_rt,      py/den_rt,      0,            0,
        -py/den,        px/den,         0,            0,
        num_1/den_p_rt, num_2/den_p_rt, px/den_rt,  py/den_rt;
  
  // Return the Jacobian matrix Hj
  return Hj;
}

VectorXd Tools::CalculateHx(const VectorXd& x_state) {
  // The h(x) vector to return
  VectorXd h(3);
  
  // Polar coordinates
  float rho;
  float phi;
  float rho_dot;
  
  // State parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Set up the h(x) vector elements
  rho = sqrt(pow(px, 2) + pow(py, 2));
  
  // Εnsure no divisions by zero in the rest of the h(x) calculations
  if (fabs(rho) < E1) {
    if (DEBUG) {cout << "Error: Division by Zero in the h(x) matrix" << endl;}
    rho = E1;
  }
  
  if (fabs(px) < E1) {
    if (DEBUG) {cout << "Error: Division by Zero in the h(x) matrix" << endl;}
    px = E1;
  }
  
  // Resulting angle φ in radians range [-pi, pi]
  phi = atan2(py, px);
  rho_dot = ((px * vx) + (py * vy)) / rho;
  
  // Initialize h(x) with the polar coordinates: ρ, φ, ρ'
  h << rho, phi, rho_dot;
  
  // Return the h(x) function
  return h;
}

MatrixXd Tools::GetI(const VectorXd& x_state) {
  long x_size = x_state.size();
  return MatrixXd::Identity(x_size, x_size);
}

float Tools::wrapMax(float x, float max) {
  return fmod(max + fmod(x, max), max);
}

float Tools::wrapMinMax(float x, float min, float max) {
  return min + wrapMax(x - min, max - min);
}
