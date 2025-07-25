#include "Simulation.h"
#include <cmath>
#include <algorithm>

    // Implement the simulation function


    std::array<std::vector<double>,3> simulateRocketDynamics(){
    // Constants
    const double g = 9.8066;      // gravitational acceleration (m/s^2)
    const double m = 50.0;        // mass of the rocket (kg)
    const double T = 7000.0;      // thrust force (N) for phase 1
    const double rho = 1.225;     // air density at sea level (kg/m^3)
    const double Cd = 0.237;      // drag coefficient (dimensionless)
    const double A = 0.0314;      // cross-sectional area of the rocket (m^2)
    const double burn_time = 5.0; // time thrust is active (seconds)
    const double total_time = 50.0; // total simulation time (seconds)
    const double dt = 0.01;       // time step (seconds)
    const double v0 = 0.0;        // initial velocity (m/s)
    const double gamma0 =80*M_PI/180; // initial flight path angle (radians)
    unsigned int i=0;
    // Initial position
    double x0 = 0.0;
    double h0 = 0.0;

    // Drag force function
    auto drag_force = [rho, Cd, A](double v) {
        return 0.5 * rho * v * v * Cd * A;
    };

    // Rocket dynamics function
    auto rocket_dynamics = [g, m, T, burn_time, drag_force](double t, const Eigen::Vector4d& state) {


        double h = state(3);

        double v = state(0);

        double gamma = (h>2)?state(1):(M_PI_4);

        double x = state(2);


        double thrust = (t <= burn_time) ? T : 0.0;
        double D = drag_force(v);

        double dvdt = (thrust / m) - (D / m) - g * sin(gamma);
        double dgamdt = (v != 0) ? (-g * cos(gamma)) / v : 0.0;
        double dxdt = v * cos(gamma);
        double dhdt = v * sin(gamma);

        return Eigen::Vector4d(dvdt, dgamdt, dxdt, dhdt);
    };

    std::vector<double> time_steps;
    for (double t = 0.0; t <= total_time; t += dt) {
        time_steps.push_back(t);
    }

    size_t n = time_steps.size();
    std::vector<double> a(n, 0.0);
    // RK4 step function
    auto rk4_step = [dt, &rocket_dynamics,&a](double t, const Eigen::Vector4d& state,size_t i) {
        Eigen::Vector4d k1 = rocket_dynamics(t, state);
        a[i]=k1.x();
        Eigen::Vector4d k2 = rocket_dynamics(t + dt / 2.0, state + dt / 2.0 * k1);
        Eigen::Vector4d k3 = rocket_dynamics(t + dt / 2.0, state + dt / 2.0 * k2);
        Eigen::Vector4d k4 = rocket_dynamics(t + dt, state + dt * k3);
        Eigen::Vector4d newstate=state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        return newstate ;
    };

    // Initialize time and state variables

    std::vector<double> v(n, 0.0);
    std::vector<double> gamma(n, 0.0);
    std::vector<double> x(n, 0.0);
    std::vector<double> h(n, 0.0);


    // Initial conditions
    v[0] = v0;
    gamma[0] = gamma0;
    x[0] = x0;
    h[0] = h0;
    a[0]=0;

    // Time integration using RK4
    size_t j = n; // Default to n in case the rocket doesn't hit the ground during simulation
    for (size_t i = 1; i < n; ++i) {
        double t = time_steps[i - 1];
        Eigen::Vector4d state(v[i - 1], gamma[i - 1], x[i - 1], h[i - 1]);

        Eigen::Vector4d new_state = rk4_step(t, state,i);

        v[i] = new_state(0);
        gamma[i] = std::clamp(new_state(1), -M_PI / 2.0, M_PI / 2.0);
        x[i] = new_state(2);
        h[i] = new_state(3);

        if (i!=0&&h.at(i)<h.at(i-1)) {
            j = i;
            break; // Rocket has hit the ground
        }
    }

    // Resize vectors to the point where the rocket hits the ground
    v.resize(j);
    gamma.resize(j);
    x.resize(j);
    h.resize(j);

    // Assign the computed x and h to the output parameters
    return std::array<std::vector<double>,3>{gamma,v,a};
}
