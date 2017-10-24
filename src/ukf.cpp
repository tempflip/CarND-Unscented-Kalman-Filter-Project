#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
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

  if (is_initialized_ != true) {

    cout << "INITING" << endl;
    lastTimestamp = meas_package.timestamp_;
    is_initialized_ = true;

    Init(meas_package);

  } else {

    ////////////////////
    ////////////////////
    ////////////////////
    ////////////////////    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) { return; }
    ////////////////////
    ////////////////////
    ////////////////////
    ////////////////////



    float dt = (meas_package.timestamp_ - lastTimestamp) /1000000.;
    lastTimestamp = meas_package.timestamp_;
    cout << "#### dt " << dt << endl;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //cout << "R :::::::::::: " << meas_package.raw_measurements_ << endl;
    Predict_State_Sigma_Points();

  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    cout << "L :::::::::::: " << meas_package.raw_measurements_ << endl;



    x_(0) = meas_package.raw_measurements_(0);
    x_(1) = meas_package.raw_measurements_(1);
  }
  }

  cout << "-------------------------------------------------" << endl;
}


void UKF::Init(MeasurementPackage meas_package) {
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
    cout << "IM INITED!!!!!" << endl;
  }

  n_x_ = 5;
  lambda_ =  3 - n_x_;


}


void UKF::Predict_State_Sigma_Points() {
  Xsig_pred_ = Predict_Sigma_Points(n_x_, P_, x_);
  cout << "Sigmas OK" << Xsig_pred_ << endl;

}

MatrixXd UKF::Predict_Sigma_Points(int dim, MatrixXd P, VectorXd x) {

  //create sigma point matrix
  MatrixXd sigmas = MatrixXd(dim, 2 * dim + 1);

  
  MatrixXd A = P.llt().matrixL();

  //set first column of sigma point matrix
  sigmas.col(0) = x;
  
  //set remaining sigma points
  for (int i = 0; i < dim; i++) {
    sigmas.col(i + 1)     = x + sqrt(lambda_ + dim) * A.col(i);
    sigmas.col(i + 1 + dim) = x - sqrt(lambda_ +dim) * A.col(i);
  }
  
  return sigmas;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
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
}



