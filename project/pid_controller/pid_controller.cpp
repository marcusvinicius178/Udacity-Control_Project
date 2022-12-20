/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
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
   output_limi_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   current_cte = cte;
   // I guess I need to create an array to calculate the diff_cte and the int_cte....in this way I get
   //diff_cte = current_cte - prev_cte
   //int_cte = 0
   //int_cte += current_cte
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi] 
    // From Step 2 of Udacity the output of controller must be set to [-1,1]
   */
    double control;
    // MUST CALCULATE THE PID ERROR HERE!!! BELOW:
    // The total error is the sum of each cte, I guess it means the INTEGRAL ERROR?
    // MUST i CALCULATE HERE ??????????????????????? OR IN UpdateError() function above?
    
    // From Step 2 of Udacity the output of controller must be set to [-1,1]
    control = (-Kp*current_cte) + (-Kd*current_cte) + (-Ki*current_cte);
    if (control < output_lim_min){
    	cout << "Control Output (torque) is TOO LOW, adjusting to minimum value = " << output_lim_min << " \n ";
    	control = output_lim_min
    } else if (control > output_lim_max){
    	cout << "Control Output (torque) is TOO HIGH, setting it to maximum value = " << output_lim_max << " \n ";
    	control = output_lim_max
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
}
