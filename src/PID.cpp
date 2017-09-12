#include <cmath>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    numOfReadings = 0;
    tol = 0.2; // As suggested in lesson
    step_id = 1;
    param_id = 0;
    best_err = 10000; // Considering that, cte > 2.6 takes car out of road

    prob_param[0] = 0.1;
    prob_param[1] = 1;
    prob_param[2] = 0.01;
}

PID::~PID() {}

void PID::Twiddle(double err)
{
    err = fabs(err);
    best_err = fabs(best_err);

    double sum = prob_param[0] + prob_param[1] + prob_param[2];
    if(sum > tol)
    {
        switch(step_id)
        {
            case 1 :
                params[param_id] += prob_param[param_id];
                step_id = 2;
                break;

            case 2 :
                if (err < best_err)
                {
                    best_err = err;
                    prob_param[param_id] *= 1.1;
                }
                else
                {
                    params[param_id] -= 2 * prob_param[param_id];
                }
                step_id = 3;
                break;

            case 3 :
                if (err < best_err)
                {
                    best_err = err;
                    prob_param[param_id] *= 1.1;
                }
                else
                {
                    params[param_id] += prob_param[param_id];
                    prob_param[param_id] *= 0.9;
                }
                param_id++;
                step_id = 1;
                break;

            default:
                step_id = 1;
                param_id = 0;
                break;
        }

        if(param_id > 2) // for 3 params - 0, 1, 2
        {
            param_id = 0;
        }

        Kp = params[0];
        Kd = params[1];
        Ki = params[2];
    }
}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    params[0] = Kp;
    params[1] = Kd;
    params[2] = Ki;
}

void PID::UpdateError(double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error = i_error + cte;
}

double PID::TotalError()
{
    return (-Kp * p_error -Kd * d_error - Ki * i_error);
}

double PID::LimitVal(double max, double min, double val)
{
    if (val > max)
    {
        val = max;
    }
    if (val < min)
    {
        val = min;
    }
    return val;
}







