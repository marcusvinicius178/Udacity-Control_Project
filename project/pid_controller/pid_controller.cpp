/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 15, 2022
 *      Author: Marcus Vinícius
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   Kp = Kpi;      // The Kp Proportional coefficient from pid_controller.h file receives the user input value = Kpi
   Ki = Kii;	 // The Ki Integral coefficient from pid_controller.h file receives the user input value = Kii
   Kd = Kdi;    //The Kd Derivative coefficient from pid_controller.h file receives the input value = Kdi 
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   //current_cte = 0.0; // Defined yet on pid_controller.h
   // It seems the errors must be initalized here and not on pid_controller.cpp as related here https://knowledge.udacity.com/questions/820447
   proportional_error = 0.0;
   differential_error = 0.0;
   integral_error = 0.0;
   
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

   // Differential Error
   if (delta_time>0){ // We must check if delta_time = 0 or not to avoid error in divistion, according to this link
 
  	differential_error = (cte - proportional_error) /delta_time; // cte will always be the current error and proportional the previous one
   }
   else{
   	differential_error = 0.0;
   	}
   // Proportional Error and next previous proportional error (from differential_error)
   proportional_error = cte; // Otherwise if you do diff_cte -=cte it will be always equal = 0
   
   // Integral Error;
   integral_error += cte*delta_time;  
   
   
   
  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi] 
    // From Step 2 of Udacity the output of controller must be set to [-1,1]
   */
    double control;

    //control = (-Kp*current_cte) + (-Kd*diff_cte) + (-Ki*int_cte);
    control = (Kp*proportional_error + Kd*differential_error + Ki*integral_error);
    if (control < output_lim_min){
    	cout << "Control Output (torque) is TOO LOW, adjusting to minimum value = " << output_lim_min << " \n ";
    	control = output_lim_min;
    } else if (control > output_lim_max){
    	cout << "Control Output (torque) is TOO HIGH, setting it to maximum value = " << output_lim_max << " \n ";
    	control = output_lim_max;
    }
    else {
    	cout << "Control Output (torque) is between the accetable boundaries [ " << output_lim_min << "," << output_lim_min << " output_lim_max ]";  
   	cout << "Contol Output = " << control;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
   return delta_time;
}
