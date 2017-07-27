#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdlib.h>

namespace MPC_TUNING_PARAMS {
    // Set the timestep length and duration
    extern const size_t TIME_STEP_DURATION;
    extern const double DT;
    extern const double DT_LATENCY;

    // Cost penalties for cte, epsi and v_start
    extern const double LAMBDA_CTE;
    extern const double LAMBDA_EPSI;
    extern const double LAMBDA_V;

    // Additional hyperparameters to penalize agressive maneuvers
    // Cost penalties to minimize use of steering and acceleration
    extern const double LAMBDA_DELTA;
    extern const double LAMBDA_A;
    // Cost penalties to minimize sudden changes in steering and acceleration
    extern const double LAMBDA_DDELTA;
    extern const double LAMBDA_DA;

    // Length from front to CoG that has a similar radius.
    extern const double LF;

    // Reference (target steady state) velocity
    extern const double REF_CTE;
    extern const double REF_EPSI;
    extern const double REF_V;

    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    extern const size_t X_START;
    extern const size_t Y_START;
    extern const size_t PSI_START;
    extern const size_t V_START;
    extern const size_t CTE_START;
    extern const size_t EPSI_START;
    extern const size_t DELTA_START;
    extern const size_t A_START;

}

#endif /* PARAMS_H_ */
