#include <cstdlib>
#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen/Dense"
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
  Tools tools; // Object to use helper functions
  MatrixXd y; // Error vector
  
  // Kalman Filter Measurement update step
  y = z - (H_ * x_);
  
  // Call the remaining Kalman Filter equations
  UpdateMatrices(y);
}

/**
 * Update the state by using Kalman Filter equations for the Radar sensor
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Tools tools; // Object to use helper functions
  MatrixXd y; // Error vector
  VectorXd h(3); // The h(x) function for the EKF
  
  // In order to calculate the error y for a radar measurement, we use the
  // h(x) function to map the cartesian coordinates to polar coordinates
  // to match the z measurement of the radar that is given in polar as well.
  h = tools.CalculateHx(x_);
  
  // Calculate the error y and normalize the y(phi) in the [-π, π] range
  y = z - h;
  y(1) = tools.wrapMinMax(y(1), -M_PI, M_PI);
  
  // Call the remaining Kalman Filter equations
  UpdateMatrices(y);
}

void KalmanFilter::UpdateMatrices(const VectorXd &y) {
  Tools tools; // Object to use helper functions
  MatrixXd S;
  MatrixXd K; // Kalman gain
  MatrixXd I = tools.GetI(x_); // Identity matrix
  
  // For the Radar, H_ is set as the Jacobian matrix is used to calculate
  // S, K and P_
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();
  
  // New state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
