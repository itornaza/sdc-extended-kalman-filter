#include "FusionEKF.h"
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "tools.h"
#include "constants.h"

using namespace std;
using namespace Constants;
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
  
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
  
  // Initialize the state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1,  0,  0,    0,
              0,  1,  0,    0,
              0,  0,  1000, 0,
              0,  0,  0,    1000;
  
  // Object to use the helper methods
  Tools tools;
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
  //------------------
  // Initialization
  //------------------
  
  // Initialize the Kalman filter with the first measurement
  // provided either from Radar or lidar
  if (!is_initialized_) {
    // Define the filter
    ekf_.x_ = VectorXd(4);
    float px = 0;
    float py = 0;
    float vx = 0;
    float vy = 0;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR
        && RADAR_ON) {
      // Get the radar measurements from the pack
      float rho = measurement_pack.raw_measurements_[0];      // ρ
      float phi = measurement_pack.raw_measurements_[1];      // φ
      float rho_dot = measurement_pack.raw_measurements_[2];  // ρ'
      
      // Convert the position from polar to cartesian coordinates
      // Discard the velocity because it is the radial velocity which
      // differs from the object velocity
      // Note: radar radial velocity (ρ') differs from the object's velocity
      // v, so we set it to zero
      px = rho * cos(phi);
      py = rho * sin(phi);
      
      // Initialize only on the very first measurement
      if (DEBUG) { cout << "Radar initialization measurement" << endl; }
      is_initialized_ = true;
      
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER
               && LASER_ON) {
      // Get the location coordinates from the pack
      // Note: lidar does not provide velocity measurements
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      
      // Initialize only on the very first measurement
      if (DEBUG) { cout << "Lidar initialization measurement" << endl; }
      is_initialized_ = true;
    } else {
      if (DEBUG) { cout << "All sensors are off" << endl; }
      is_initialized_ = true;
    }

    // Avoid the px = 0 and py = 0 case for initialization
    if (fabs(px) < E1 && fabs(py) < E1) {
      if (DEBUG) { cout << "Initialization with px = py = 0" << endl; }
      px = E1;
      py = E1;
    }
    
    // Initialize the kalman filter with the current position and zero velocity
    ekf_.x_ << px, py, vx , vy;
    
    // Update the previous timestamp with the initial measurement timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // Exit without executing the prediction and update steps
    return;
  }

  //------------------
  // Prediction
  //------------------
  
  // Compute the time elapsed between the current and previous measurements
  // dt is expressed in seconds where the pack provides msecs
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / MSEC_TO_SEC;
  
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
    (dt_4/4)*noise_ax, 0,                 (dt_3/2)*noise_ax, 0,
    0,                 dt_4/4*noise_ay,   0,                 (dt_3/2)*noise_ay,
    (dt_3/2)*noise_ax, 0,                 dt_2*noise_ax,     0,
    0,                 (dt_3/2)*noise_ay, 0,                 dt_2*noise_ay;
  
  // Kalman Filter Predict step
  ekf_.Predict();

  //------------------
  // Update
  //------------------

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR
      && RADAR_ON) {
    // Radar updates (using Hj and h(x))
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER
             && LASER_ON) {
    // Laser updates (using H)
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
  // Print the output
  if (VERBOSE) {
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
  }
}
