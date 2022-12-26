/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 15, 2022
 *      Author: Marcus Vinícius
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
    double proportional_error;       // Crostrack Error (Proportional Term)
    double differential_error;  // Derivative Cross Track Error
    double integral_error;    // Integral Cross Track Error

// Looking to this issue https://knowledge.udacity.com/questions/820447, it seems these variables values must be initialized on pid_controller.cpp, not here!

    /*
    * Coefficients
    */
    double Kp;       // Proportional Gain
    double Ki;       // Integral Gain
    double Kd;      // Derivative Gain
    

    /*
    * Output limits
    */
    
    double output_lim_max;  // Max Tolerance Threshold
    double output_lim_min;  // Min Tolerance Threshold
  
    /*
    * Delta time
    */
    double delta_time;     // No idea what this delta time is
    
    /*
    * Gain Vectors
    */
    vector<double> gains_ (3); // Size is 3 paramenters;
    vector<double> increment_gains_ (3);
    
    

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    
    void PID::TwiddleGainsUp(vector<double> gains, vector<double> increment_gains);
    
    /*
    * Tune PID gains Up
    */
    
    void PID::TwiddleGainsDown(vector<double> gains, vector<double> increment_gains);
    
    /*
    * Tune PID gains Down
    */
    
    void PID::TwiddleShrinkGains(vector<double> gains, vector<double> increment_gains);
    
    /*
    * Restore Original Gains of PID & Shrink the Incremental Gains Boundaries
    */
    
    
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H
