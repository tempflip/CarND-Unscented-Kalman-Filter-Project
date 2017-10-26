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
  //std_a_ = 30;
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;
  std_yawdd_ = 0.2;

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
      //Predict_State_Sigma_Points();

      //cout << "Lovely new predicted sigmas OK" << Xsig_pred_ << endl;

      Predict_Augmented_Sigma_Points();
      Predict_New_Sigma_Points(dt);
      Predict_P();
      cout << "P ::::::::" << P_ << endl;
      cout << "x ::::::::" << x_ << endl;
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
  //Xsig_pred_ = Generate_Sigma_Points(n_x_, P_, x_);
  //cout << "Sigmas OK" << Xsig_pred_ << endl;

}

void UKF::Predict_Augmented_Sigma_Points() {
  int aug_dim = 7;
  //create augmented mean vector
  VectorXd x_aug = VectorXd(aug_dim);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(aug_dim, aug_dim);
  //create sigma point matrix
  //MatrixXd sigmas = MatrixXd(aug_dim, 2 * aug_dim + 1);

  //create augmented mean state
   x_aug(0) = x_(0,0);
   x_aug(1) = x_(1,0);
   x_aug(2) = x_(2,0);
   x_aug(3) = x_(3,0);
   x_aug(4) = x_(4,0);
   x_aug(5) = 0;
   x_aug(6) = 0;  
  //create augmented covariance matrix
  MatrixXd Q = MatrixXd(2,2);
  Q(0,0) = std_a_ * std_a_;
  Q(1,1) = std_yawdd_ * std_yawdd_;
  
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = Q(0,0);
  P_aug(6,6) = Q(1,1);

  aug_sigmas_ = Generate_Sigma_Points(aug_dim, P_aug, x_aug);

  //cout << "##### AUG SIGMAS " << aug_sigmas_ << endl;



}

MatrixXd UKF::Generate_Sigma_Points(int dim, MatrixXd P, VectorXd x) {

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

void UKF::Predict_New_Sigma_Points(float delta_t) {
  int n_aug = 7;
  int n_x = 5;

  Xsig_pred_ = MatrixXd(n_x, n_aug * 2 + 1);

  for (int i = 0; i< 2*n_aug+1; i++)
  {
    
    //extract values for better readability
    double p_x = aug_sigmas_(0,i);
    double p_y = aug_sigmas_(1,i);
    double v = aug_sigmas_(2,i);
    double yaw = aug_sigmas_(3,i);
    double yawd = aug_sigmas_(4,i);
    double nu_a = aug_sigmas_(5,i);
    double nu_yawdd = aug_sigmas_(6,i);


    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;

  }  
}

void UKF::Predict_P() {


  int n_aug = 7;
  //create vector for predicted state
  //VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  VectorXd weights = VectorXd(2*n_aug+1);

  double weight_0 = lambda_/(lambda_+n_aug);

  weights(0) = weight_0;


  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda_);
    weights(i) = weight;
  }

  
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights(i) * Xsig_pred_.col(i);
  }


  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
    
  }

  P_ = P;
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



