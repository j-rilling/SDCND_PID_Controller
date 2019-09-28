#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  this->err_last_cycle_initialized = false;
  this->SP = 0.0;
  this->CV = 0.0;
  this->PV = 0.0;
  this->twiddle_initialized = false;
  this->tw_coeff = 0;
  this->tw_step = 0;
  this->time_log_initialized = false;
  this->reset_simulator = false;
}

PID::~PID() {}

void PID::InitCoeffs(double Kp_, double Ki_, double Kd_, double p_change_, double i_change_, double d_change_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  this->p_change = p_change_;
  this->i_change = i_change_;
  this->d_change = d_change_;
}

void PID::setLimits(double SP_min_, double SP_max_, double CV_min_, double CV_max_, double pert_min_, double pert_max_) {
  this->SP_min = SP_min_;
  this->SP_max = SP_max_;
  this->CV_min = CV_min_;
  this->CV_max = CV_max_;
  this->pert_min = pert_min_;
  this->pert_max = pert_max_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your tUpdate controllerotal error calc here!
}

void PID::setSP(double newSP) {
  if (newSP < this->SP_min) {
    this->SP = this->SP_min;
  }
  else if (newSP > this->SP_max) {
    this->SP = this->SP_max;
  }
  else {
    this->SP = newSP;
  }
}

double PID::updateController(double currentPV, double pert) {
  this->PV = currentPV;
  // The controller minimizes the error between the setpoint and 
  // the process value
  this->err = this->SP - this->PV;
  // If this is the first cycle of the controller, the D component needs 
  // to be initialized
  if (!this->err_last_cycle_initialized) {
    this->err_last_cycle = this->err;
    this->err_last_cycle_initialized = true;
  }
  // D part of the controller
  double diff_err = this->err - this->err_last_cycle;
  // I part of the controller
  this->err_sum += this->err;
  // Corresponds to the control error on the last cycle, which will be used
  // on the next cycle
  this->err_last_cycle = this->err;

  // The new CV is calculated with the three control components
  double newCV = (this->Kp*this->err + this->Kd*diff_err + this->Ki*this->err_sum);
  // std::cout << "Control value: " << newCV_norm << std::endl;
  // double newCV = scale(newCV_norm, 0.0, 100.0, this->CV_min, this->CV_max);
  
  if (newCV < this->CV_min) {
    this->CV = this->CV_min;
  }
  else if (newCV > this->CV_max) {
    this->CV = this->CV_max;
  }
  else {
    this->CV = newCV;
  }

  return this->CV;
}

double PID::getQuadraticError(vector<double> errors) {
  double sum = 0.0;
  for (unsigned int i = 0; i < errors.size(); i++) {
    sum += pow(errors[i],2);
  }
  double quadratic_error = sum/static_cast<double>(errors.size());
  return quadratic_error;
}

void PID::twiddleOpt(double currentPV, double tolerance) {
  vector<double> coefficients {this->Kp, this->Ki, this->Kd};
  vector<double> coeff_change {this->p_change, this->i_change, this->d_change};
  double sum_diffs = this->p_change + this->i_change + this->d_change;
  // The best error is calculated at the beginning using the start parameters of the
  // PID controller
  if (!this->twiddle_initialized) {
    // To calculate the best error, it collects 200 error points.
    if (this->best_error_v.size() < 500) {
      // But first, it waits 200 points in order to ignore the transient response of
      // the controller with the new parameters
      if (this->transient_best_error_ct < 200) {
        this->transient_best_error_ct++;
      }
      else {
        this->best_error_v.push_back(this->err);
      }
    }
    else {
      // Then it calculates the best error and set the twiddle initialized flag to true
      // so the algorithm can be continued with the else if
      this->best_error = getQuadraticError(this->best_error_v);
      this->transient_best_error_ct = 0;
      this->twiddle_initialized = true;
    }
  }
  // Corresponds to the while (sum_diffs > tolerance) on the offline algorithm
  else if (this->twiddle_initialized && (sum_diffs > tolerance)) {
    // In order to select which part of the algorithm is executed on the current c
    // cycle, a switch statement is used. The variable that decides what part of the
    // algorithm is executed is "tw_step". 
    // Also, only one PID parameter is changed at the time, the current parameter being
    // changed is given by the variable "tw_coeff". This is equivalent to the for loop 
    // for the three PID parameters.
    switch (this->tw_step) {
      case 0:
        // The parameter being changed is increased by its corresponding delta and 
        // it moves to the next step.
        coefficients[this->tw_coeff] += coeff_change[this->tw_coeff];
        this->tw_step++;
        reset_simulator = true;
        break;
      case 1:
        // It collects the 200 points ignoring the transient
        if (this->error_step_1_v.size() < 500) {
          if (this->transient_error_step_1_ct < 200) {
            this->transient_error_step_1_ct++;
          }
          else {
            this->error_step_1_v.push_back(this->err);
          }
        }
        // It gets the quadratic error, clears the "error_step_1_v" vector to be 
        // used on a next cycle, sets also the counter "transient_error_step_1_ct" to 0
        // and moves on too the next step. 
        else {
          this->error_step_1 = getQuadraticError(this->error_step_1_v);
          this->error_step_1_v.clear();
          this->tw_step++;
          this->transient_error_step_1_ct = 0;
        }
        break;
      case 2:
        // If the current calculated error is smaller than the best error, the answer is accepted
        // and the best error gets replaced by the new calculated error. The corresponding delta is
        // multiplied by 1.1 making it bigger. The index for the parameter being changed is incremented 
        // and "tw_step" is set to 0 in order to start from the beginning for the next parameter. 
        if (this->error_step_1 < this->best_error) {
          this->best_error = this->error_step_1;
          coeff_change[tw_coeff] *= 1.1;
          this->tw_step = 0;
          this->tw_coeff = (this->tw_coeff + 1)%3; 
        }
        // Otherwise it moves on to the next step.
        else {
          this->tw_step++;
        }
        break;
      case 3:
        // The parameter being changed is decreased by twice its delta (once for the increment done before and
        // once to decrease the original parameter)
        coefficients[this->tw_coeff] -= 2.0*coeff_change[this->tw_coeff];
        this->tw_step++;
        reset_simulator = true;
        break;
      case 4:
        // Again 200 points are collected ignoring the transient response and the quadratic error is calculated
        if (this->error_step_2_v.size() < 500) {
          if (this->transient_error_step_2_ct < 200) {
            this->transient_error_step_2_ct++;
          }
          else {
            this->error_step_2_v.push_back(this->err);
          }
        }
        // It gets the quadratic error, clears the "error_step_2_v" vector to be 
        // used on a next cycle, sets also the counter "transient_error_step_2_ct" to 0
        // and moves on too the next step. 
        else {
          this->error_step_2 = getQuadraticError(this->error_step_2_v);
          this->error_step_2_v.clear();
          this->tw_step++;
          this->transient_error_step_2_ct = 0;
        }
        break;
      case 5:
        // If the current calculated error is smaller than the best error, the answer is accepted
        // and the best error gets replaced by the new calculated error. The corresponding delta is
        // multiplied by 1.1 making it bigger. The index for the parameter being changed is incremented 
        // and "tw_step" is set to 0 in order to start from the beginning for the next parameter. 
        if (this->error_step_2 < this->best_error) {
          this->best_error = this->error_step_2;
          coeff_change[tw_coeff] *= 1.1;
          this->tw_step = 0;
          this->tw_coeff = (this->tw_coeff + 1)%3;
        }
        // Otherwise it moves on to the next step
        else {
          this->tw_step++;
        }
        break;
      case 6:
        // If none of the parameter changing gave a smaller error, the original parameter is restored and 
        // the corresponding delta is multiplied by 0.9 making it smaller, so the resolution for the search
        // is made higher. The index for the parameter being changed is incremented and "tw_step" is set to
        // 0 in order to start from the beginning for the next parameter.
        coefficients[this->tw_coeff] += coeff_change[this->tw_coeff];
        coeff_change[this->tw_coeff] *= 0.9;
        this->tw_step = 0;
        this->tw_coeff = (this->tw_coeff + 1)%3;
        break;     
    }
  }
  // The real parameters of the class are updated
  this->Kp = coefficients[0];
  this->Ki = coefficients[1];
  this->Kd = coefficients[2];
  this->p_change = coeff_change[0];
  this->i_change = coeff_change[1];
  this->d_change = coeff_change[2];
}

void PID::printParameters() {
  std::cout << "Kp: " << this->Kp;
  std::cout << ", Ki: " << this->Ki;
  std::cout << ", Kd: " << this->Kd;
  std::cout << ", dKp: " << this->p_change;
  std::cout << ", dKi: " << this->i_change;
  std::cout << ", dKd: " << this->d_change;
  std::cout << ", Best squared error: " << this->best_error << std::endl;
}

/**
 * Print SP, PV and CV, best error, Kp, Ki, Kd, dKp, dKi and dKd to a 
 * CSV file to be processed and plotted afterwards in Python.
 */
void PID::printToCSV(string filename) {
  if (!this->time_log_initialized) {
    this->start_miliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
                              (std::chrono::system_clock::now().time_since_epoch()).count();
    this->time_log_initialized = true;
  }
  long int current_miliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
                                (std::chrono::system_clock::now().time_since_epoch()).count();

  double time_from_start = static_cast<double>(current_miliseconds - start_miliseconds);

  std::ofstream outputFile;
  outputFile.open(filename.c_str(), std::ios::out | std::ios::app);
  if (outputFile.fail()) {
    std::cout << "Log failed to open" << std::endl;
  }
  outputFile << time_from_start << " ";
  outputFile << this->SP << " ";
  outputFile << this->PV << " ";
  outputFile << this->CV << " ";
  outputFile << this->best_error << " ";
  outputFile << this->Kp << " ";
  outputFile << this->Ki << " ";
  outputFile << this->Kd << " ";
  outputFile << this->p_change << " ";
  outputFile << this->i_change << " ";
  outputFile << this->d_change << std::endl;
  outputFile.close();
}

double PID::scale(double in, double in_min, double in_max, double out_min, double out_max) {
  double out;
  if (in < in_min) {
    out = out_min;
  }
  else if (in > in_max) {
    out = out_max;
  }
  else {
    double m = (out_max - out_min)/(in_max - in_min);
    out = m*(in - in_min) + out_min;
  }
  return out;
}