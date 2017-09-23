#include <cstdlib>
#include <iostream>
#include <cmath>
#include "kalman_filter.h"
#include "tools.h"
#include "constants.h"

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
  // u is zero so we don't use it in x = F * x + u
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * Update the state by using Kalman Filter equations for the Lidar sensor
 */
void KalmanFilter::Update(const VectorXd &z) {
  // Local variables
  MatrixXd S;
  MatrixXd Si;
  MatrixXd y; // Error
  MatrixXd K; // Kalman gain
  MatrixXd I; // Identity matrix
  MatrixXd Ht = H_.transpose();
  
  // Set up the identity matrix
  int x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  
  // Kalman Filter Measurement update step
  y = z - (H_ * x_);
  
  S = H_ * P_ * Ht + R_;
  Si = S.inverse();
  K = P_ * Ht * Si;
  
  // New state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

/**
 * Update the state by using Kalman Filter equations for the Radar sensor
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //------------------
  // Local variables
  //------------------
  
  // Vectors and Matrices
  MatrixXd S;
  MatrixXd Si;
  MatrixXd y; // Error
  MatrixXd K; // Kalman gain
  VectorXd h(3); // To convert cartesian to polar coordinates
  MatrixXd Ht = H_.transpose();
  
  // Identity matrix
  MatrixXd I;
  long x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);
  
  // Polar coordinates
  float rho;
  float phi;
  float rho_dot;
  
  // State parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  //--------
  // h(x)
  //--------
  
  // Set up the h(x) vector elements
  rho = sqrt(pow(px, 2) + pow(py, 2));
  
  // Εnsure no divisions by zero in the rest of the h(x) calculations
  if (fabs(rho) < E1 || fabs(px) < E1) {
    cout << "Error: Division by Zero in the h(x) matrix" << endl;
    
    // TODO: replace with rho = E and px = E  ????
    exit(EXIT_FAILURE);
  }
  
  // Resulting angle φ in radians range [-pi, pi]
  phi = atan(py / px);
  rho_dot = ((px * vx) + (py * vy)) / rho;
  
  // Initialize h(x) with the polar coordinates: ρ, φ, ρ'
  h << rho, phi, rho_dot;
  
  //----------------------------------------
  // Kalman Filter Measurement update step
  //----------------------------------------
  
  // In order to calculate the error y for a radar measurement, we use the
  // h(x) function to map the cartesian coordinates to polar coordinates
  // to match the z measurement of the radar that is given in polar as well.
  y = z - h;
  
  // H_ is set as the Jacobian matrix is used to calculate S, K and P_
  S = H_ * P_ * Ht + R_;
  Si = S.inverse();
  K = P_ * Ht * Si;
  
  // New state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
