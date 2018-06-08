#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;

  // State dimension
  n_x_ = 5;
  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // time when the state is true, in us
  time_us_ = 0;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  // weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);

  NIS_laser_ = 0.0;

  NIS_radar_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /**********************
   * Initialization
   **********************/

  if(!is_initialized_){

    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      // Lidar values
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);

      x_ << px, py, 0, 0, 0;
    }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      // Convert from polar to cartisian
      double ro = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);

      double px = ro*cos(phi);
      double py = ro*sin(phi);

      x_ << px, py, 0, 0, 0;
    }

    P_ << 0.15, 0, 0, 0, 0,
       0, 0.15, 0, 0, 0,
       0, 0, 1, 0, 0,
       0, 0, 0, 1, 0,
       0, 0, 0, 0, 1;

    // Save the first timestamp in seconds
    time_us_ = meas_package.timestamp_;
    // Done initialization
    is_initialized_ = true;
    return;
  }

  /**********************
   * Prediction
   **********************/

  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  /**********************
   * Update
   **********************/

  if(use_laser_ && meas_package.sensor_type_==MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }else if(use_radar_ && meas_package.sensor_type_==MeasurementPackage::RADAR){
    UpdateRadar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}
/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // create augmented mean state
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  MatrixXd Xsig_aug(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;
  for(int i=0; i<n_aug_; i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);
  }

  // predict sigma points
  for(int i=0; i<2*n_aug_+1; i++){
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if(fabs(yawd) > 0.0001){
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    }else{
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // wirte predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i=1; i<2*n_aug_+1; i++){
    weights_(i) = 0.5/(lambda_+n_aug_);
  }

  // predicted state mean
  x_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // 2x1 vector: [px, py]
  VectorXd z = meas_package.raw_measurements_;
  // Measurement matrix
  MatrixXd H(2, 5);
  H << 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0;
  // Measurement noise
  MatrixXd R(2, 2);
  R << std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  // 5x5 indentity matrix
  MatrixXd I = MatrixXd::Identity(5, 5);

  VectorXd y = z - H*x_;
  MatrixXd S = H*P_*H.transpose() + R;
  MatrixXd K = P_*H.transpose()*S.inverse();

  NIS_laser_ = y.transpose()*S.inverse()*y;

  x_ = x_ + K*y;
  P_ = (I - K*H) * P_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // 3x1 vector: [ro, phi, rod]
  VectorXd z = meas_package.raw_measurements_;
  // matrix for sigma points in measurement space
  MatrixXd Zsig(3, 2*n_aug_+1);

  // transform sigma points into measurement space
  for(int i=0; i<2*n_aug_+1; i++){
    // extract values for better readability
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = v*cos(yaw);
    double v2 = v*sin(yaw);

    // measurement model
    Zsig(0, i) = sqrt(px*px + py*py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px*v1 + py*v2) / Zsig(0, i);
  }

  // mean predict measurement
  VectorXd z_pred = VectorXd::Zero(3);
  for(int i=0; i<2*n_aug_+1; i++){
    z_pred += weights_(i) * Zsig.col(i);
  }

  // predicted covariance matrix
  MatrixXd S = MatrixXd::Zero(3, 3);
  // cross-correlation matrix
  MatrixXd Tc = MatrixXd::Zero(5, 3);

  for(int i=0; i<2*n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // add measurement noise
  MatrixXd R(3, 3);
  R << std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0, std_radrd_*std_radrd_;
  S += R;

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;

  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();

}
