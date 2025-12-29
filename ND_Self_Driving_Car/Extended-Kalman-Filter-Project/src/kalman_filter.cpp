#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Tools tools;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in,Eigen::MatrixXd &H_j, Eigen::MatrixXd &R_laser,Eigen::MatrixXd &R_radar, Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_laser_ = R_laser;
  R_radar_ = R_radar;
  Q_ = Q_in;
  Hj_= H_j;
}

void KalmanFilter::Predict() {
  /**
   *  predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   *  update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
   x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   *  update the state by using Extended Kalman Filter equations
   */
  
  
  VectorXd z_pred = tools.convert_cartesian_to_polar(x_);
  VectorXd y = z - z_pred;
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  if (y.size() == 3) y(1) = atan2(sin(y(1)), cos(y(1)));
  
  //new estimate
   x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
  
}


void KalmanFilter::UpdateF(const float dt)
{
  this->F_(0, 2) = dt;
  this->F_(1, 3) = dt;

}


void KalmanFilter::UpdateQ(const MatrixXd& Qin)
{
  this->Q_=Qin;
}

void KalmanFilter::UpdateJacobai(const Eigen::MatrixXd& hj)
{
  this->Hj_ = hj;
}