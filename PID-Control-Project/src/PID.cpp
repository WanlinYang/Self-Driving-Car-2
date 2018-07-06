#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {

  // Initialize coefficients
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;

  // Initialize errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  p_error = cte;

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if(i_error > 0.5)
    i_error = 0.5;
  else if(i_error < -0.5)
    i_error = -0.5;

}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

