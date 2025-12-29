#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  
  this->P = MatrixXd(4, 4);
  this->F = MatrixXd::Identity(4, 4);
  this->Q = MatrixXd::Zero(4, 4);


  // initializing matrices
 this->R_laser_ = MatrixXd(2, 2);
 this->R_radar_ = MatrixXd(3, 3);
 this->H_laser_ = MatrixXd(2, 4);
 this->Hj_ = MatrixXd::Zero(3, 4);
 
  
  //measurement covariance matrix - laser
  this->R_laser_ << 0.0225, 0,
              0, 0.0225;


  
  //measurement covariance matrix - radar
   this->R_radar_ << 0.09, 0, 0,
                     0, 0.0009, 0,
                     0, 0, 0.09;
 
  
  this->H_laser_  << 1, 0, 0, 0,
                     0, 1, 0, 0;
  
  this->P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0,0.0,
             0.0, 0.0, 0.0, 100;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {

       VectorXd x = VectorXd(4);
        x << 1, 1, 1, 1;
    
        MatrixXd F = MatrixXd(4, 4);
        F << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
     ekf_.Init(x,this->P, F,this->H_laser_,this->Hj_,this->R_laser_,this->R_radar_,this->Q);
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
       
       Eigen::VectorXd cartesian_cord = tools.convert_polar_to_cartesian(measurement_pack.raw_measurements_);
       float px = cartesian_cord(0);
       float py = cartesian_cord(1);
       float vx = cartesian_cord(2);
       float vy = cartesian_cord(3);
       ekf_.x_ <<px, 
                 py, 
                 vx, 
                 vy;

    previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
       ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    previous_timestamp_ = measurement_pack.timestamp_;
   
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

   
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  ekf_.UpdateF(dt);
  
  UpdateQ(dt);

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
   
    //Convert polar to cartsein coordinate system
    Eigen::VectorXd cartesian_cord = tools.convert_polar_to_cartesian(measurement_pack.raw_measurements_);
    Eigen::MatrixXd hj = tools.CalculateJacobian(cartesian_cord);
    ekf_.UpdateJacobai(hj);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    //Laser updates
        ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}


void FusionEKF::UpdateQ(const double dt)
{
 //Update process Covariance matrix
 float dt4_noise_ax = (pow(dt,4)/4)*noise_ax;
 float dt3_noise_ax = (pow(dt,3)/2)*noise_ax;
 float dt2_noise_ax = (pow(dt,2))*noise_ax;
  
 float dt4_noise_ay = (pow(dt,4)/4)*noise_ay;
 float dt3_noise_ay = (pow(dt,3)/2)*noise_ay;
 float dt2_noise_ay = (pow(dt,2))*noise_ay;
 
 this->Q << dt4_noise_ax, 0, dt3_noise_ax , 0,
            0, dt4_noise_ay, 0, dt3_noise_ay,
            dt3_noise_ax, 0, dt2_noise_ax, 0,
            0, dt3_noise_ay, 0, dt2_noise_ay;
 
  ekf_.UpdateQ( this->Q);   

}