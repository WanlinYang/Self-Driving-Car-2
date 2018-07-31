#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
int N = 10;
double dt = 0.1;

double v_ref = 30.0;
int x_start = 0;
int y_start = N;
int psi_start = 2*N;
int v_start = 3*N;
int cte_start = 4*N;
int epsi_start = 5*N;
int delta_start = 6*N;
int a_start = 6*N + (N-1);

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
  public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    fg[0] = 0;

    for(int t=0; t<N; t++){
      fg[0] += 3000*CppAD::pow(vars[cte_start+t], 2);
      fg[0] += 3000*CppAD::pow(vars[epsi_start+t], 2);
      fg[0] += CppAD::pow(vars[v_start+t]-v_ref, 2);
    }

    for(int t=0; t<N-1; t++){
      fg[0] += CppAD::pow(vars[delta_start+t], 2);
      fg[0] += CppAD::pow(vars[a_start+t], 2);
    }

    for(int t=0; t<N-2; t++){
      fg[0] += 200*CppAD::pow(vars[delta_start+t+1]-vars[delta_start+t], 2);
      fg[0] += 10*CppAD::pow(vars[a_start+t+1]-vars[a_start+t], 2);
    }

    // initial state, bump up 1
    fg[x_start+1] = vars[x_start];
    fg[y_start+1] = vars[y_start];
    fg[psi_start+1] = vars[psi_start];
    fg[v_start+1] = vars[v_start];
    fg[cte_start+1] = vars[cte_start];
    fg[epsi_start+1] = vars[epsi_start];

    for(int t=1; t<N; t++){
      // state at time t+1
      AD<double> x1 = vars[x_start+t];
      AD<double> y1 = vars[y_start+t];
      AD<double> psi1 = vars[psi_start+t];
      AD<double> v1 = vars[v_start+t];
      AD<double> cte1 = vars[cte_start+t];
      AD<double> epsi1 = vars[epsi_start+t];

      // state at time t
      AD<double> x0 = vars[x_start+t-1];
      AD<double> y0 = vars[y_start+t-1];
      AD<double> psi0 = vars[psi_start+t-1];
      AD<double> v0 = vars[v_start+t-1];
      AD<double> cte0 = vars[cte_start+t-1];
      AD<double> epsi0 = vars[epsi_start+t-1];

      // only consider the actuation at time t
      AD<double> delta0 = vars[delta_start+t-1];
      AD<double> a0 = vars[a_start+t-1];
      if(t > 1){
        delta0 = vars[delta_start+t-2];
        a0 = vars[a_start+t-2];
      }

      AD<double> f0 = coeffs[0] + x0*coeffs[1] + CppAD::pow(x0,2)*coeffs[2]
                      + CppAD::pow(x0,3)*coeffs[3];
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*x0*coeffs[2] + 3*CppAD::pow(x0,2)*coeffs[3]);

      // model constraints
      fg[x_start+t+1] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
      fg[y_start+t+1] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
      fg[psi_start+t+1] = psi1 - (psi0 - v0/Lf*delta0*dt);
      fg[v_start+t+1] = v1 - (v0 + a0*dt);
      fg[cte_start+t+1] = cte1 - ((f0-y0) + (v0*CppAD::sin(epsi0)*dt));
      fg[epsi_start+t+1] = epsi1 - ((psi0-psides0) - (v0/Lf*delta0*dt));
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
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // initial values
  double x_init = state[0];
  double y_init = state[1];
  double psi_init = state[2];
  double v_init = state[3];
  double cte_init = state[4];
  double epsi_init = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  int n_vars = 6*N + 2*(N-1);
  // TODO: Set the number of constraints
  int n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // set initial values
  vars[x_start] = x_init;
  vars[y_start] = y_init;
  vars[psi_start] = psi_init;
  vars[v_start] = v_init;
  vars[cte_start] = cte_init;
  vars[epsi_start] = epsi_init;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // What about limits for others?

  for(int i = 0; i < delta_start; i++){
    vars_lowerbound[i] = -1e19;
    vars_upperbound[i] = 1e19;
  }

  for(int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for(int i = a_start; i < n_vars; i++){
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x_init;
  constraints_lowerbound[y_start] = y_init;
  constraints_lowerbound[psi_start] = psi_init;
  constraints_lowerbound[v_start] = v_init;
  constraints_lowerbound[cte_start] = cte_init;
  constraints_lowerbound[epsi_start] = epsi_init;

  constraints_upperbound[x_start] = x_init;
  constraints_upperbound[y_start] = y_init;
  constraints_upperbound[psi_start] = psi_init;
  constraints_upperbound[v_start] = v_init;
  constraints_upperbound[cte_start] = cte_init;
  constraints_upperbound[epsi_start] = epsi_init;

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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  std::vector<double> return_sol;
  return_sol.push_back(solution.x[delta_start]);
  return_sol.push_back(solution.x[a_start]);

  for(int i=0; i<N-1; i++){
    return_sol.push_back(solution.x[x_start+i+1]);
    return_sol.push_back(solution.x[y_start+i+1]);
  }

  return return_sol;
}
