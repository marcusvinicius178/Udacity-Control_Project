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
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    if (current_cte < output_lim_mini){
    	cout << "Lower Boundary too low, setting it to minimum boundary value";
    	current_cte = output_lim_mini;
    } else if (current_cte > output_lim_maxi){
    	cout << "Upper Boundary too high, setting it to max boundary value";
    	current_cte = output_lim_maxi;
    }
    control = (-Kp*current_cte) + (-Kd*current_cte) + (-Ki*current_cte);
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
}
