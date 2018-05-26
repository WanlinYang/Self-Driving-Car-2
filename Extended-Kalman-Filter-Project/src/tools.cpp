#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::sqrt;
using std::abs;
using std::atan2;

#define PI 3.14159265

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  if(estimations.empty() || estimations.size() != ground_truth.size()){
    std::cout << "Invalid estimation or ground truth data" << std::endl;
    return VectorXd::Zero(4);
  }

  // initialize rmse as zero vector
  int dim = ground_truth[0].size();
  VectorXd rmse = VectorXd::Zero(dim);

  // accumulate squared residuals
  for(unsigned int i=0; i<estimations.size(); i++){
    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // pre-compute
  double c1 = px*px + py*py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;

  // check division by zero
  if(abs(c1) < 0.0001){
    std::cout << "CalculateJacobian() Error - Division by zero."
              << std::endl;
    return Hj;
  }

  // compute Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
				-(py/c1), (px/c1), 0, 0,
				py*(vx*py-vy*px)/c3, px*(px*vy-py*vx)/c3, px/c2, py/c2;

  return Hj;
}

VectorXd Tools::CalculateMeasurement(const VectorXd& x_state){

  VectorXd measurement(3);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double c = sqrt(px*px + py*py);
  if(abs(c) < 0.0001){
    std::cout << "CalculateMeasurement() Error - Division by zero."
              << std::endl;
    return measurement;
  }

  measurement << c, atan2(py,px), (px*vx+py*vy)/c;
  return measurement;
}

double Tools::AdjustAngle(const double& angle){
  if(angle < PI && angle > -PI)
    return angle;

  double result = angle;

  while(result > PI){
    result -= 2*PI;
  }
  while(result < -PI){
    result += 2*PI;
  }

  return result;
}
