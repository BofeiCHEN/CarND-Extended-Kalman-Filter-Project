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
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3,4);
   // state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   float p_x_y = pow(px, 2) + pow(py, 2); // x*x + y*y
   float sqrt_pxy = sqrt(p_x_y); // sqrt( x*x + y*y)
   float p_15_x_y = p_x_y*sqrt_pxy; // pow(p_x_y, 1.5)

   if(fabs(p_x_y) < 0.00001){
        cout << "CalculateJacobian() - Error - Division by zero" <<endl;
        return Hj;
    }

   Hj << px/sqrt_pxy,   py/sqrt_pxy,   0,     0, 
         -py/p_x_y,       px/p_x_y,    0,     0,
         (py*(vx*py - vy*px))/p_15_x_y,   (px*(vy*px - vx*py))/p_15_x_y,   px/sqrt_pxy,   py/sqrt_pxy;
   
   return Hj;
}
