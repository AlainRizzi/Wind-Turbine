function theta=turbine_dynamics(beta,R,V,dt,T_final,b)
% Parameters
rho = 1.225;       % Air density (kg/m^3)
% R = 10;            % Rotor radius (m)
 J = 10000;         % Moment of inertia (kg*m^2)
% beta = 1;          % Pitch angle (deg), can be changed as input
% V = 10;            % Wind speed (m/s), can be changed dynamically

% Load torque function (example linear load)
T_load = @(omega) 100 * omega;  

% Power coefficient Cp function
Cp = @(lambda, beta) 0.5176*(116./lambda - 0.4*beta - 5).*exp(-21./lambda);

% Simulation parameters
%dt = 0.01;         % Time step (s)
%T_final = 20000;      % Total simulation time (s)
N = (T_final/dt);

% Initialize states: omega and theta
omega = zeros(1, N+1);
theta = zeros(1, N+1);

% Initial conditions
omega(1) = 2;    % initial angular velocity (rad/s)
theta(1) = 0;      % initial blade angle (rad)


% Main RK4 integration loop
for i = 1:N
    y = [omega(i); theta(i)];
    
    k1 = turbine_dynamics1(y, V, beta, J, R, rho, Cp, T_load,b);
    k2 = turbine_dynamics1(y + 0.5*dt*k1, V, beta, J, R, rho, Cp, T_load,b);
    k3 = turbine_dynamics1(y + 0.5*dt*k2, V, beta, J, R, rho, Cp, T_load,b);
    k4 = turbine_dynamics1(y + dt*k3, V, beta, J, R, rho, Cp, T_load,b);
    
    y_next = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    omega(i+1) = y_next(1);
    theta(i+1) = y_next(2);
end
theta
%omega
% b
% V
% R
% beta
% Cp
end
% Plot results
%time = 0:dt:T_final;
%figure;
%subplot(2,1,1);
% plot(time, omega);
% xlabel('Time (s)');
% ylabel('\omega (rad/s)');
% title('Rotor Angular Velocity');
% 
% subplot(2,1,2);
% plot(time, theta);
% xlabel('Time (s)');
% ylabel('\theta (rad)');
% title('Rotor Angular Position');
% Dynamics function for ODE
function dydt = turbine_dynamics1(y, V, beta, J, R, rho, Cp, T_load,b)
    omega = y(1);
    theta = y(2);
    if omega < 0.1
        omega = 0.1; % avoid division by zero and divergence in tip speed ratio
    end
    lambda = (omega * R) / V;
    lambda_i = 1 / (1/(lambda + 0.08*beta) - 0.035/(beta^3 + 1));
    Cp_val = max(Cp(lambda_i, beta), 0);
    A = pi * R^2;
    T_aero = 0.5 * rho * A * Cp_val * V^3 / omega;
    T_friction=b*omega;
    domega = (T_aero - T_load(omega)-T_friction) / J;
    dtheta = omega;
    dydt = [domega; dtheta];
end