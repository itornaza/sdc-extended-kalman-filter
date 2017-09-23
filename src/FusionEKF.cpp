#include "FusionEKF.h"
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "tools.h"
#include "constants.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Constructor
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement function matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;
}

/**
 * Destructor
 */
FusionEKF::~FusionEKF() {}

/**
 * Responsible for the initialization of the Kalman filter as well as calling 
 * the prediction and update steps of the Kalman filter
 */
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack){
  if (DEBUG) { cout << "Enter FusionEKF::ProcessMeasurement()" << endl; }
  
  //------------------
  // Initialization
  //------------------
  
  // Initialize the Kalman filter with the first measurement
  // provided either from Radar or lidar
  if (!is_initialized_) {
    if (DEBUG) { cout << "First measurement" << endl; }
    
    // Define the filter
    ekf_.x_ = VectorXd(4);
    float px = 0;
    float py = 0;
    float vx = 0;
    float vy = 0;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Get the radar measurements from the pack
      float rho = measurement_pack.raw_measurements_[0];      // ρ
      float phi = measurement_pack.raw_measurements_[1];      // φ
      float rho_dot = measurement_pack.raw_measurements_[2];  // ρ'
      
      // Convert the position from polar to cartesian coordinates
      // Discard the velocity because it is the radial velocity which
      // differs from the object velocity
      px = rho * cos(phi);
      py = rho * sin(phi);
      
      // Note: radar radial velocity (ρ') differs from the object's velocity
      // v, so we set it to zero
      
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Get the location coordinates from the pack
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      
      // Note: lidar does not provide velocity measurements
    }

    // Initialize the kalman filter with the current position and zero velocity
    ekf_.x_ << px, py, vx , vy;
    
    // TODO: FIX
    // Deal with the special case initialisation problems
    if (fabs(ekf_.x_(0)) < E1 && fabs(ekf_.x_(1)) < E1){
      ekf_.x_(0) = E1;
      ekf_.x_(1) = E1;
    }
    
    // Initialize the state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1,  0,  0,    0,
                0,  1,  0,    0,
                0,  0,  1000, 0,
                0,  0,  0,    1000;
    
    // Update the previous timestamp with the initial measurement timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // Done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  //------------------
  // Prediction
  //------------------
  
  // Compute the time elapsed between the current and previous measurements
  // dt is expressed in seconds where the pack provides msecs
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / MSEC_TO_SEC;
  if (DEBUG) { cout << "dt = " << dt << endl; }
  
  // Update the previous timestamp with the measurement timestamp
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Modify the F matrix so that the time is integrated
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0,  dt,
              0, 0, 1,  0,
              0, 0, 0,  1;
  
  // Object acceleration noise components for the Q matrix calculations
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  
  // Calculate the powers of dt to use in the Q matrix
  float dt_2 = pow(dt, 2);
  float dt_3 = pow(dt, 3);
  float dt_4 = pow(dt, 4);
  
  // Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<
    (dt_4/4)*noise_ax,  0,               (dt_3/2)*noise_ax, 0,
    0,                  dt_4/4*noise_ay, 0,                 (dt_3/2)*noise_ay,
    (dt_3/2)*noise_ax, 0,                dt_2*noise_ax,     0,
    0,               (dt_3/2)*noise_ay,  0,                 dt_2*noise_ay;
  
  // Kalman Filter Predict step
  ekf_.Predict();

  //------------------
  // Update
  //------------------

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    
    // TODO: DEBUG the UpdateEKF to avoid the segmentation fault
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
