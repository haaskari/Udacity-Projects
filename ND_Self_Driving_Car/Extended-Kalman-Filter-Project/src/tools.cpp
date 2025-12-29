#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truths) {
  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  for (int k = 0; k < estimations.size(); ++k){

    VectorXd diff = estimations[k] - ground_truths[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
 
    MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
   std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  
}


Eigen::VectorXd Tools::convert_polar_to_cartesian(const Eigen::VectorXd& v)
{
  VectorXd cartesian_vec(4);

  const double rho = v(0);
  const double phi = v(1);
  const double drho = v(2);

  const double px = rho * cos(phi);
  const double py = rho * sin(phi);
  const double vx = drho * cos(phi);
  const double vy = drho * sin(phi);

  cartesian_vec << px, py, vx, vy;
  return cartesian_vec;

}



 Eigen::VectorXd Tools::convert_cartesian_to_polar(const Eigen::VectorXd& v)
 {
  const double THRESH = 0.0001;
  VectorXd polar_vec(3);

  const double px = v(0);
  const double py = v(1);
  const double vx = v(2);
  const double vy = v(3);

  const double rho = sqrt( px * px + py * py);
  const double phi = atan2(py, px); //accounts for atan2(0, 0)
  const double drho = (rho > THRESH) ? ( px * vx + py * vy ) / rho : 0.0;

  polar_vec << rho, phi, drho;
  return polar_vec;
 }