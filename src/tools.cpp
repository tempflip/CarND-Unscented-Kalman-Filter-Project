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

  VectorXd RMSE = VectorXd(4);
  RMSE << 0,0,0,0;

  
  for (int i = 0; i < estimations.size(); ++i) {
  	VectorXd diff = estimations[i] - ground_truth[i];
  	diff = diff.array() * diff.array();
  	diff = diff.array().sqrt();
  	RMSE = RMSE + diff;
  }

  RMSE = RMSE / estimations.size(); 
  return RMSE;


  return RMSE;
}