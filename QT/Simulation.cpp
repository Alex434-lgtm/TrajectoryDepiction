#include "Simulation.h"
#include <cmath>
#include <algorithm>

//---------------------------------------
// Rocket Dynamics Simulation
//---------------------------------------
std::array<std::vector<double>,3> simulateRocketDynamics(){
    // Physical constants
    const double g = 9.8066;      // Gravitational acceleration (m/s^2)
    const double m = 50.0;        // Mass of the rocket (kg)
    const double T = 7000.0;      // Thrust force (N) during burn phase
    const double rho = 1.225;     // Air density at sea level (kg/m^3)
    const double Cd = 0.237;      // Drag coefficient (dimensionless)
    const double A = 0.0314;      // Cross-sectional area of the rocket (m^2)
    const double burn_time = 5.0; // Engine burn duration (seconds)
    const double total_time = 50.0; // Total simulation time (seconds)
    const double dt = 0.01;       // Time step for integration (seconds)
    const double v0 = 0.0;        // Initial velocity (m/s)
    const double gamma0 = 80*M_PI/180; // Initial flight path angle (radians)
    
    // Initial position coordinates
    double x0 = 0.0;
    double h0 = 0.0;

    // Aerodynamic drag force calculation
    auto drag_force = [rho, Cd, A](double v) {
        return 0.5 * rho * v * v * Cd * A;
    };

    // Rocket dynamics differential equations
    auto rocket_dynamics = [g, m, T, burn_time, drag_force](double t, const Eigen::Vector4d& state) {
        // Extract state variables
        double v = state(0);      // Velocity
        double gamma = state(1);  // Flight path angle
        double x = state(2);      // Horizontal position
        double h = state(3);      // Vertical position

        // Prevent flight path angle from going negative at low altitude
        if (h <= 2) {
            gamma = M_PI_4;  // Force 45 degree angle near ground
        }

        // Calculate thrust (active only during burn time)
        double thrust = (t <= burn_time) ? T : 0.0;
        
        // Calculate drag force opposing motion
        double D = drag_force(v);

        // Differential equations for rocket motion
        double dvdt = (thrust / m) - (D / m) - g * sin(gamma);  // Velocity change
        double dgamdt = (v != 0) ? (-g * cos(gamma)) / v : 0.0; // Flight path angle change
        double dxdt = v * cos(gamma);                            // Horizontal velocity
        double dhdt = v * sin(gamma);                            // Vertical velocity

        return Eigen::Vector4d(dvdt, dgamdt, dxdt, dhdt);
    };

    // Generate time steps for simulation
    std::vector<double> time_steps;
    for (double t = 0.0; t <= total_time; t += dt) {
        time_steps.push_back(t);
    }

    size_t n = time_steps.size();
    
    // Initialize acceleration array for tracking
    std::vector<double> a(n, 0.0);
    
    // Runge-Kutta 4th order integration
    auto rk4_step = [dt, &rocket_dynamics, &a](double t, const Eigen::Vector4d& state, size_t i) {
        // RK4 algorithm implementation
        Eigen::Vector4d k1 = rocket_dynamics(t, state);
        a[i] = k1.x();  // Store acceleration for visualization
        
        Eigen::Vector4d k2 = rocket_dynamics(t + dt / 2.0, state + dt / 2.0 * k1);
        Eigen::Vector4d k3 = rocket_dynamics(t + dt / 2.0, state + dt / 2.0 * k2);
        Eigen::Vector4d k4 = rocket_dynamics(t + dt, state + dt * k3);
        
        // Weighted average of derivatives
        Eigen::Vector4d newstate = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        return newstate;
    };

    // Initialize state variable arrays
    std::vector<double> v(n, 0.0);      // Velocity array
    std::vector<double> gamma(n, 0.0);  // Flight path angle array
    std::vector<double> x(n, 0.0);      // Horizontal position array
    std::vector<double> h(n, 0.0);      // Vertical position array

    // Set initial conditions
    v[0] = v0;
    gamma[0] = gamma0;
    x[0] = x0;
    h[0] = h0;
    a[0] = 0;

    // Time integration loop using RK4
    size_t j = n; // Default to full simulation length
    
    for (size_t i = 1; i < n; ++i) {
        double t = time_steps[i - 1];
        Eigen::Vector4d state(v[i - 1], gamma[i - 1], x[i - 1], h[i - 1]);

        // Perform RK4 step
        Eigen::Vector4d new_state = rk4_step(t, state, i);

        // Update state variables
        v[i] = new_state(0);
        gamma[i] = std::clamp(new_state(1), -M_PI / 2.0, M_PI / 2.0); // Limit angle to ±90°
        x[i] = new_state(2);
        h[i] = new_state(3);

        // Check if rocket has reached apogee (descending)
        if (i != 0 && h.at(i) < h.at(i-1)) {
            j = i;
            break; // Stop simulation at apogee
        }
    }

    // Resize arrays to actual simulation length
    v.resize(j);
    gamma.resize(j);
    x.resize(j);
    h.resize(j);

    // Return flight path angle, velocity, and acceleration arrays
    return std::array<std::vector<double>,3>{gamma, v, a};
}