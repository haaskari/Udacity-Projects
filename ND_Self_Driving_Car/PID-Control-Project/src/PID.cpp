#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  last_cte = 0.0;
  count = 0;
  sumofError = 0.0;

}

void PID::UpdateError(double cte) {
    p_error = cte;

  // Integral error.
  i_error += cte;

  // Diferential error.
  d_error = cte - last_cte;
  last_cte = cte;

  sumofError += cte;
  count++;


}

double PID::TotalError() {
   return p_error * Kp + i_error * Ki + d_error * Kd;
  
}