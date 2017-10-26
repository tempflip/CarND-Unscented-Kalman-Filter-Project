#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  bool is_initialized_;
  bool use_laser_;
  bool use_radar_;
  VectorXd x_;
  MatrixXd P_;
  MatrixXd Xsig_pred_;
  long long time_us_;
  double std_a_;
  double std_yawdd_;
  double std_laspx_;
  double std_laspy_;
  double std_radr_;
  double std_radphi_;
  double std_radrd_ ;
  VectorXd weights_;
  int n_x_;
  int n_aug_;
  int n_sig_;
  double lambda_;
  double NIS_radar_;
  double NIS_laser_;
  MatrixXd R_radar_;
  MatrixXd R_lidar_;
  UKF();

  virtual ~UKF();


  void Init(MeasurementPackage measurement_pack);
  void NormAng(double *ang);
  void ProcessMeasurement(MeasurementPackage measurement_pack);
  void Prediction(double delta_t);
  void UpdateLidar(MeasurementPackage meas_package);
  void UpdateRadar(MeasurementPackage meas_package);
  void UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z);
};

#endif 