#ifndef PID_H
#define PID_H

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

using std::vector;
using std::string;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID coefficients
   * @param (Kp_, Ki_, Kd_, p_change_, i_change_, d_change_) The initial PID coefficients
   */
  void InitCoeffs(double Kp_, double Ki_, double Kd_, double p_change_, double i_change_, double d_change_, double deadband_);


  /**
   * Initialize PID limits
   * @param (SP_min_, SP_max_, CV_min_, CV_max_) The initial limits
   */
  void setLimits(double SP_min_, double SP_max_, double CV_min_, double CV_max_);

  /**
   * Set Setpoint to the desired value
   */
  void setSP(double newSP);

  /**
   * Executes the PID Controller algorithm and returns the control value
   * which is used on the actuator
   */
  double updateController(double currentPV);

  /**
   * Online twiddle optimization algorithm. It updates the PID parameters
   * based on the squared error of the last squared error values.
   */
  void twiddleOpt(double currentPV, double tolerance, unsigned int paramOptimized);


  /**
   * Calculates the quadratic error
   */
  double getQuadraticError(vector<double> errors);

  /**
   * Prints parameters of controller
   */
  void printParameters();

  /**
   * Print SP, PV and CV, best error, Kp, Ki, Kd, dKp, dKi and dKd to a 
   * CSV file to be processed and plotted afterwards in Python.
   */
  void printToCSV(string filename);


  /**
   * This variable resets the Udacity simulator in order to make the exact same 
   * simulation with new parameters when using the twiddle optimization
   */
  bool reset_simulator;

 private:
  /**
   * PID Errors
   */
  double p_change;
  double i_change;
  double d_change;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  double Kp_best;
  double Ki_best;
  double Kd_best;


  /**
   * PID Variables
   */
  double SP; // Setpoint
  double PV; // Process value 
  double CV; // Control value
  double err; // Error value (SP - PV)

  double err_last_cycle; // Error value on the last cycle (used for differential part of controller)
  bool err_last_cycle_initialized; // Indicates if the error on the last cycle was already initialized or not
  double err_sum;   // Sum of all last errors (used for integral part of controller)

  bool twiddle_initialized; 

  unsigned int tw_coeff;  // Indicates to the twiddle optimization algorithm what coefficient to process
                          // on the current cycle
  unsigned int tw_step;   // Indicates to the twiddle optimization algorithm what step to perform on the
                          // current cycle

  vector<double> best_error_v;  // Vector of error values used to calculate the best error at the beginning of the simulation
  double best_error;            // Best quadratic error, calculated at the beginning of the simulation and then reduced when 
                                // a better parameter is found.
  unsigned int transient_best_error_ct; // Counter for the first points with these parameters, used in order to ignore the
                                        // transient response of the controller.
  vector<double> error_step_1_v;  // Vector of error values used to calculate the current error by adding the delta to the current
                                  // parameter.
  double error_step_1;            // Calculated quadratic error when adding the delta to the parameter now being processed
  unsigned int transient_error_step_1_ct; // Counter for the first points to avoid the transient response

  vector<double> error_step_2_v;  // Vector of error values used to calculate the current error by substracting the delta to the current
                                  // parameter.
  double error_step_2;            // Calculated quadratic error when substracting the delta to the parameter now being processed
  unsigned int transient_error_step_2_ct; // Counter for the first points to avoid the transient response

  bool time_log_initialized;
  long int start_miliseconds;

  long int current_time_ms;
  long int last_cycle_time_ms;

  double dt;

  double SP_min;
  double SP_max;
  double CV_min;
  double CV_max;
  double deadband;

};

#endif  // PID_H