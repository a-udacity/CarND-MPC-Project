#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "mpc_tuning_params.h"

using CppAD::AD;
using namespace MPC_TUNING_PARAMS;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {

        // Implementation of MPC - similar to MPC quiz implementation
        // `fg` a vector of the cost constraints
        // `vars` is a vector of variable values (state & actuators)

        // Cost (optimization objective) is stored in fg[0].
        fg[0] = 0.0;
        // Augment cost with error: cte, epsi, (v-ref_v)
        for (size_t t = 0; t < TIME_STEP_DURATION; t++) {
            fg[0] += LAMBDA_CTE * CppAD::pow(vars[CTE_START + t] - REF_CTE, 2);
            fg[0] += LAMBDA_EPSI * CppAD::pow(vars[EPSI_START + t] - REF_EPSI, 2);
            fg[0] += LAMBDA_V * CppAD::pow(vars[V_START + t] - REF_V, 2);
            // Augment cost with large inputs in steering
            if (t < TIME_STEP_DURATION - 1) {
                fg[0] += LAMBDA_DELTA * CppAD::pow(vars[DELTA_START + t], 2);
                fg[0] += LAMBDA_A * CppAD::pow(vars[A_START + t], 2);
            }
            if (t < TIME_STEP_DURATION - 2) {
                // Augment cost with consecutively large changes or inputs in
                // steering and acceleration/brake(deceleration)
                // penalize sharp turns and acceleration/decelration events
                fg[0] += LAMBDA_DDELTA * CppAD::pow(vars[DELTA_START + t + 1] - vars[DELTA_START + t], 2);
                fg[0] += LAMBDA_DA * CppAD::pow(vars[A_START + t + 1] - vars[A_START + t], 2);
            }
        }

        // Setup Constraints
        // Note: Index offset by 1 as fg[0] is reserved for cost (objective)
        // Constraints on initial values
        fg[1 + X_START] = vars[X_START];
        fg[1 + Y_START] = vars[Y_START];
        fg[1 + PSI_START] = vars[PSI_START];
        fg[1 + V_START] = vars[V_START];
        fg[1 + CTE_START] = vars[CTE_START];
        fg[1 + EPSI_START] = vars[EPSI_START];

        // The rest of the constraints
        for (size_t t = 1; t < TIME_STEP_DURATION; t++) {
            AD<double> x1 = vars[X_START + t];
            AD<double> y1 = vars[Y_START + t];
            AD<double> psi1 = vars[PSI_START + t];
            AD<double> v1 = vars[V_START + t];
            AD<double> cte1 = vars[CTE_START + t];
            AD<double> epsi1 = vars[EPSI_START + t];

            AD<double> x0 = vars[X_START + t - 1];
            AD<double> y0 = vars[Y_START + t - 1];
            AD<double> psi0 = vars[PSI_START + t - 1];
            AD<double> v0 = vars[V_START + t - 1];
            AD<double> cte0 = vars[CTE_START + t - 1];
            AD<double> epsi0 = vars[EPSI_START + t - 1];

            AD<double> delta0 = vars[DELTA_START + t - 1];
            AD<double> a0 = vars[A_START + t - 1];

            AD<double> f0 = coeffs[0] + coeffs[1] * x0
                            + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0
                                             + 3.0 * coeffs[3] * x0 * x0);

            // Rest of the constraints based on Global Kinematics Model (GKM)
            fg[1 + X_START + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
            fg[1 + Y_START + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
            fg[1 + PSI_START + t] = psi1 - (psi0 + v0 * delta0 / LF * DT);
            fg[1 + V_START + t] = v1 - (v0 + a0 * DT);
            fg[1 + CTE_START + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
            fg[1 + EPSI_START + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / LF * DT);
        }

    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // Total number of variables (6 state and 2 control)
    size_t n_vars = 6 * TIME_STEP_DURATION + 2 * (TIME_STEP_DURATION - 1);
    // Total number of constraints
    //size_t n_objectives = 1;
    size_t n_constraints = 6 * TIME_STEP_DURATION;
    //size_t n_fgsize = n_objectives+n_constraints;

    // Initial value of the independent variables.
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (i = 0; i < DELTA_START; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (i = DELTA_START; i < A_START; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/deceleration upper and lower limits.
    // The unit measure in Unity is 1m, so acceleration is m/s2.
    // An LB/UB of 1 m/s2 is tight for longitudinal accel./decel.
    for (i = A_START; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Set constraints for initial state
    constraints_lowerbound[X_START] = x;
    constraints_lowerbound[Y_START] = y;
    constraints_lowerbound[PSI_START] = psi;
    constraints_lowerbound[V_START] = v;
    constraints_lowerbound[CTE_START] = cte;
    constraints_lowerbound[EPSI_START] = epsi;

    constraints_upperbound[X_START] = x;
    constraints_upperbound[Y_START] = y;
    constraints_upperbound[PSI_START] = psi;
    constraints_upperbound[V_START] = v;
    constraints_upperbound[CTE_START] = cte;
    constraints_upperbound[EPSI_START] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values in vector format.

    vector<double> actuatorValues;
    actuatorValues = {};
    for (i = 0; i < solution.x.size(); i++) {
        actuatorValues.push_back(solution.x[i]);
    }
    return actuatorValues;

}
