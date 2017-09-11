#ifndef PID_H
#define PID_H

class PID {
public:

    int numOfReadings;
    int step_id;
    int param_id;
    double prob_param[3];
    double params[3];
    double tol;
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double best_err;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double LimitVal(double max, double min, double val);

  void Twiddle(double cte);
};

#endif /* PID_H */
