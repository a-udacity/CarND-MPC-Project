#include "mpc_tuning_params.h"

namespace MPC_TUNING_PARAMS {
    // Set the time step length and duration
    const double DT = 0.2;
    const double DT_LATENCY = 0.1;

    // Cost penalties for cte, epsi and v_start
    const double LAMBDA_CTE = 1.0;
    const double LAMBDA_EPSI = 1.0;
    const double LAMBDA_V = 1.0;

    // Additional hyper parameters to penalize aggresive maneuvers
    // Following lambda's are Lagrange multipliers for the optimizer
    // Cost penalties to minimize use of steering and acceleration
    const double LAMBDA_DELTA = 1.0;
    const double LAMBDA_A = 1.0;
    // Cost penalties to minimize sudden changes in steering and acceleration
    const double LAMBDA_DDELTA = 5000.0;
    const double LAMBDA_DA = 1.0;

    // Length from front to CoG that has a similar radius.
    const double LF = 2.67;

    // Reference (target steady state) velocity
    const double REF_CTE = 0.0;
    const double REF_EPSI = 0.0;
    const double REF_V = 60.0 * 0.44704; // convert mph to m/s

    const size_t TIME_STEP_DURATION = 10;
    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    const size_t X_START = 0;
    const size_t Y_START = X_START + TIME_STEP_DURATION;
    const size_t PSI_START = Y_START + TIME_STEP_DURATION;
    const size_t V_START = PSI_START + TIME_STEP_DURATION;
    const size_t CTE_START = V_START + TIME_STEP_DURATION;
    const size_t EPSI_START = CTE_START + TIME_STEP_DURATION;
    const size_t DELTA_START = EPSI_START + TIME_STEP_DURATION;
    const size_t A_START = DELTA_START + TIME_STEP_DURATION - 1;

}