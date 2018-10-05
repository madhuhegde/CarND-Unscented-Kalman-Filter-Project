#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() 
{
  mse  =   VectorXd(4);
  alpha =  VectorXd(4);
  beta  =  VectorXd(4);
  rmse  =   VectorXd(4);
  
  mse << 0, 0, 0, 0;
  alpha << 0.02, 0.02, 0.01, 0.01;
  
  /* beta = (1-alpha) */
  beta << 0.98, 0.98, 0.99, 0.99;
  
}

Tools::~Tools() {}

// Calculate the RMSE
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 

{
  int index = estimations.size();    
  

  VectorXd res = estimations[index-1] - ground_truth[index-1];
  
  res = res.array() * res.array();
  res = res.array() * alpha.array();
  
  mse = mse.array() * beta.array();
  mse = res + mse;
    


  // Calculate the RMSE
  rmse = mse.array().sqrt();

  if( rmse(0) > .10 ||
      rmse(1) > .11 ||
      rmse(2) > .40 ||
      rmse(3) > .30 )
    cout << "Warning  " << ":  rmse = " 
         << rmse(0) << "  " << rmse(1) << "  " 
         << rmse(2) << "  " << rmse(3) << endl;
    
  return rmse;
}
 
