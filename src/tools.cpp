#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0,0,0,0;

   // check inputs: estimation vector size != zero; estimation vector size == ground truth vector size
   if(0 == estimations.size()){
      cout << "Estimation vector size is 0!" << endl;
      return rmse;
   }
  
   if( ground_truth.size() != estimations.size()){
      cout << "Ground truth size is not equal to estimation vector size!" << endl;
      return rmse;
   }

   // accumulate squared residuals
   for (unsigned int i=0; i < estimations.size(); i++) {

      VectorXd residual = estimations[i] - ground_truth[i];

      // coefficient-wise multiplication
      // rmse += residual.array()*residual.array(); Why don't work?
      residual = residual.array()*residual.array();
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
   * TODO:
   * Calculate a Jacobian here.
   */
}
