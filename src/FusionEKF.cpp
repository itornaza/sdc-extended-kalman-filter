#include "FusionEKF.h"
#include <iostream>
#include "tools.h"
#include "Eigen/Dense"

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
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

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
    
    // Done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  //------------------
  // Prediction
  //------------------

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // Noise due to acceleration
  float noise_ax_ = 9.0;
  float noise_ay_ = 9.0;
  
  ekf_.Predict();

  //------------------
  // Update
  //------------------

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
