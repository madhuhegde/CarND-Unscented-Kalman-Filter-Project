#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "tools.h"
#include "ukf.h"
#include <fstream>
#include <sstream>

using namespace std;


int main()
{
  uWS::Hub h;
  
  // Create a Kalman Filter instance
  UKF ukf;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  vector<string> sensor_type_str;
  vector<double> NIS_value;
  vector<VectorXd> RMSE_value;

  // hardcoded input/output file with laser and radar measurements
  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  string out_file_name_ = "../data/obj_pose-laser-radar-synthetic-output.txt";
  ifstream in_file(in_file_name_.c_str(),std::ifstream::in);
  ofstream out_file(out_file_name_.c_str(), ofstream::out);

    string line;
    // set i to get only first 6 measurments
    //int i = 0;
    while(getline(in_file, line))// && (i<=6))
    {
          //i = i+1;
       MeasurementPackage meas_package;
       istringstream iss(line);
       long long timestamp;
        
       // reads first element from the current line
       string sensor_type;
       iss >> sensor_type;

       if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
              
        } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
        }
            
        if(((sensor_type.compare("L") == 0) && (ukf.use_laser_)) ||
             ((sensor_type.compare("R") == 0) && (ukf.use_radar_)))
        {
              
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
        
        
          ground_truth.push_back(gt_values);

          //Call ProcessMeasurment(meas_package) for Kalman filter
    	  ukf.ProcessMeasurement(meas_package);
        
          sensor_type_str.push_back(sensor_type);
          if (sensor_type.compare("L") == 0)
          {
            NIS_value.push_back(ukf.NIS_laser_);
          }
        
          if (sensor_type.compare("R") == 0)
          {
            NIS_value.push_back(ukf.NIS_radar_);
          }
        
    	  //Push the current estimated x,y positon from the Kalman filter's state vector

    	  VectorXd estimate(4);

    	  double p_x = ukf.x_(0);
    	  double p_y = ukf.x_(1);
    	  double v  = ukf.x_(2);
    	  double yaw = ukf.x_(3);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
    	  estimations.push_back(estimate);

          /* Compute RMSE */
          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          RMSE_value.push_back(RMSE);
       }
        
    }
    
    /* Store output into a file - obj_pose-laser-radar-synthetic-output.txt */
    out_file << "Sensor" << "\t";
    out_file << "px_state" << "\t";
    out_file << "py_state" << "\t";
    out_file << "vx_state" << "\t";
    out_file << "vy_state" << "\t";
    out_file << "NIS_value" << "\t";
    out_file << "RMSE[0]" << "\t";
    out_file << "RMSE[1]" << "\t";
    out_file << "RMSE[2]" << "\t";
    out_file << "RMSE[3]" << "\n";
    
    size_t number_of_measurements = sensor_type_str.size();
    out_file << std::fixed;
    out_file << std::setprecision(3);
    
    for (size_t k = 0; k < number_of_measurements; ++k)
    {
        VectorXd estimate = estimations[k];
        VectorXd RMSE = RMSE_value[k];
        
        out_file <<"  "<< sensor_type_str[k] << "\t\t";
        out_file << estimate[0] << "\t\t";
        out_file << estimate[1] << "\t\t";
        out_file << estimate[2] << "\t\t";
        out_file << estimate[3] << "\t\t";
        out_file << NIS_value[k] << "\t\t";
        out_file << RMSE[0]   << "\t";
        out_file << RMSE[1]   << "\t";
        out_file << RMSE[2]   << "\t";
        out_file << RMSE[3]   << "\n";

    }
    if(in_file.is_open()){
        in_file.close();
    }
    
    // close files
    if (out_file.is_open()) {
        out_file.close();
    }
    
}























































































