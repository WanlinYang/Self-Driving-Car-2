#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

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
  int dim = ground_truth.size();
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
