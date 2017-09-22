#include <cstdlib>
#include <iostream>
#include <cmath>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
 * Predict the state for both sensor types using the Kalman Filter Equations
 */
void KalmanFilter::Predict() {
  x_ = (F_ * x_); // u_ is zero so we don't use it [ x_ = (F_ * x_) + u_ ]
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * Update the state by using Kalman Filter equations for the Lidar sensor
 */
void KalmanFilter::Update(const VectorXd &z) {
  // Local variables
  MatrixXd S;
  MatrixXd y; // Error
  MatrixXd K; // Kalman gain
  MatrixXd I; // Identity matrix
  
  // Set up the identity matrix
  int x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  
  // Kalman Filter Measurement update step
  y = z - (H_ * x_);
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();
  
  // New state
  x_ = x_ + (K * y);
  P_ = (I - (K * H_)) * P_;
}

/**
 * Update the state by using Kalman Filter equations for the Radar sensor
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Local variables
  MatrixXd S;
  MatrixXd y;     // Error
  MatrixXd K;     // Kalman gain
  VectorXd h(3);  // Converts cartesian to polar coordinates
  MatrixXd I;     // Identity matrix
  
  float e = 0.001; // Small positive number to compare to zero
  
  // Polar coordinates
  float rho;
  float phi;
  float rho_dot;
  
  // Recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // Set up the identity matrix
  long x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  
  // Set up the h vector elements
  rho = sqrt(pow(px, 2) + pow(py, 2));
  
  // Validate input, ensure no divisions by zero in the h(x) calculations
  if (fabs(rho) < e || fabs(px) < e) {
    cout << "Error: Division by Zero in the h(x) matrix" << endl;
    exit(EXIT_FAILURE);
  }
  
  phi = atan(py / px); // Resulting angle in radians range [-pi, pi]
  rho_dot = ((px * vx) + (py * vy)) / rho;
  
  // Initialize h(x) with (ρ, φ, ρ')
  h << rho, phi, rho_dot;
  
  // Kalman Filter Measurement update step
  
  // In order to calculate the error y for a radar measurement, we use the
  // h(x) function to map the cartesian coordinates to polar coordinates
  // to match the z measurement of the radar that is given in polar as well.
  y = z - h;
  
  // H_ is set as the Jacobian matrix is used to calculate S, K and P_
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();
  
  // New state
  x_ = x_ + (K * y);
  P_ = (I - (K * H_)) * P_;
}
