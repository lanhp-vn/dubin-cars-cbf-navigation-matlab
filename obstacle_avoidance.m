%% Dubins Car Simulation with Standstill and Optional Dither
% This script simulates the navigation of a single Dubins car model from a
% starting point to a goal while avoiding a circular obstacle. It uses a
% safety-critical controller based on High-Order Control Barrier Functions
% (HOCBF) formulated as a Quadratic Program (QP). The simulation includes
% features for stopping at the goal (standstill) and adding optional dither
% to the control input to potentially escape local minima.

% Clear the workspace of all variables, close all open figures, and clear
% the command window to ensure a clean run.
clear;
close all;
clc;

%% Initialization
% This section defines all the core parameters for the simulation, robot,
% environment, and controllers.

% --- Simulation and Robot Configuration ---
number_of_robots = 1; % The number of robots in the simulation.
time = 0; % Initialize simulation time to zero.
sampling_time = 1e-1; % Set the discrete time step for the simulation (s).

% --- State and Goal Definition ---
number_of_states = 3 * number_of_robots; % Total number of state variables.
states = [0, 2, 0]'; % Initial state vector [x-pos; y-pos; heading] (m, m, rad).
goal_states = [10, 8, 0]'; % Final goal state vector (m, m, rad).

% --- Robot Kinematics and Control ---
linear_velocity = 1; % Set the constant forward speed of the robot (m/s).
number_of_inputs = 1 * number_of_robots; % Total number of control inputs.
max_min_control_input = 3; % Symmetric control bounds for the steering input (rad/s).

% --- Environment Definition ---
obstacle_position = [5, 5]'; % Center position of the circular obstacle [x; y] (m).
obstacle_radius = 2; % Radius of the circular obstacle (m).
% Generate points for plotting the circular obstacle.
x_obstacle = obstacle_position(1) + obstacle_radius * cos(linspace(0, 2*pi));
y_obstacle = obstacle_position(2) + obstacle_radius * sin(linspace(0, 2*pi));

% --- Controller Parameters ---
% Control Barrier Function (CBF) gain.
class_K_const = 1;
% Distance from the goal at which the simulation will stop.
standstill_threshold = 5e-2;
% Optional dither (noise) to add to the control signal.
use_dither = true; % Set to 'true' to enable dither, 'false' to disable.
dither_amplitude = 0.1; % Amplitude of the random dither noise.

%% Preallocation
% Pre-allocate arrays to store historical data for later analysis and plotting.
state_vector_history = []; % History of the robot's state vector.
control_input_history = []; % History of the applied control input.
time_history = []; % History of simulation time steps.

%% Simulate Trajectory
% This is the main loop where the simulation is executed step-by-step.

% Initialize history logs with the starting conditions.
state_vector_history = [state_vector_history, states];
time_history = [time_history; time];
tic; % Start a timer to measure the simulation's execution time.

% The simulation runs indefinitely until the standstill condition is met.
while true
    % Check if the robot has reached the goal within the defined threshold.
    if norm(states(1:2) - goal_states(1:2)) < standstill_threshold
        disp('Target reached, stopping simulation.');
        break; % Exit the loop.
    end

    % --- Nominal Controller (Goal-Seeking Behavior) ---
    % This controller computes a desired steering angle to point the robot
    % towards the goal.
    kappa = 1; % Proportional gain for the nominal controller.
    e_theta = goal_states(1:2) - states(1:2); % Position error vector.
    % Calculate the desired steering input using a proportional controller.
    nominal_controller = kappa * (atan2(e_theta(2), e_theta(1)) - states(3));

    % --- System Dynamics (Dubins Car Model) ---
    % Defines how the robot's state changes over time. 'f' is the drift
    % term (movement without control), and 'g' maps the control input to
    % the state derivatives.
    f = [linear_velocity * cos(states(3)), linear_velocity * sin(states(3)), 0]';
    g = [0, 0, 1]';

    % --- Safety-Critical Controller (Obstacle Avoidance via HOCBF) ---
    % This controller modifies the nominal control input to guarantee safety
    % (i.e., not hitting the obstacle). It uses a High-Order Control Barrier
    % Function (HOCBF) because the control input does not directly affect
    % the robot's position (relative degree is 2).
    e_position = states(1:2) - obstacle_position; % Robot's position relative to the obstacle.
    % First-order CBF: h(x) = distance^2 - radius^2 >= 0.
    obstacle_cbf_1 = norm(e_position)^2 - obstacle_radius^2;
    % Time derivative of the first-order CBF.
    obstacle_cbf_2 = 2 * linear_velocity * (e_position(1) * cos(states(3)) + e_position(2) * sin(states(3)));
    % The HOCBF combines the first two derivatives to ensure safety.
    obstacle_hocbf = obstacle_cbf_2 + class_K_const * obstacle_cbf_1;
    % Gradient of the HOCBF with respect to the state vector.
    gradient_obstacle_hocbf = [2 * e_position(1) + 2 * linear_velocity * cos(states(3)), ...
                               2 * e_position(2) + 2 * linear_velocity * sin(states(3)), ...
                               2 * linear_velocity * (-e_position(1) * sin(states(3)) + e_position(2) * cos(states(3)))]';
    % Lie derivative of the HOCBF along the drift dynamics 'f'.
    lie_derivitive_f_obstacle_hocbf = gradient_obstacle_hocbf' * f;
    % Lie derivative of the HOCBF along the control dynamics 'g'.
    lie_derivitive_g_obstacle_hocbf = gradient_obstacle_hocbf' * g;
    % Extended class K function to ensure asymptotic stability of the safe set.
    alpha_obstacle_hocbf = class_K_const * obstacle_hocbf^3;

    % --- Quadratic Program (QP) for Optimal Safe Control ---
    % The QP finds a control input 'u' that is as close as possible to the
    % nominal controller's input, subject to the safety constraint defined
    % by the HOCBF (A*u <= b).
    uhat = reshape(nominal_controller, 1 * number_of_robots, 1);
    H = 2 * eye(1 * number_of_robots); % Quadratic cost matrix.
    z = -2 * uhat; % Linear cost vector.
    A = -lie_derivitive_g_obstacle_hocbf; % The safety constraint matrix.
    b = alpha_obstacle_hocbf + lie_derivitive_f_obstacle_hocbf; % The safety constraint vector.
    opts = optimoptions(@quadprog, 'Display', 'off'); % Suppress QP solver output.
    % Solve the QP to find the optimal safe control input.
    u_opt = quadprog(sparse(H), double(z), A, b, [], [], [], [], [], opts);

    % --- Control Input Modification ---
    % Add random noise (dither) to the control input if enabled. This can
    % help the robot escape from positions where the controller gets stuck.
    if use_dither
        u_opt = u_opt + dither_amplitude * randn();
    end

    % Saturate the final control input to respect the physical limits.
    if norm(u_opt, 'inf') > max_min_control_input
        u_star = max_min_control_input * u_opt / norm(u_opt);
    else
        u_star = u_opt;
    end

    % --- Numerical Integration (State Update) ---
    % Update the robot's state using the Forward Euler method.
    x_dot = f + g * u_star;
    states = states + x_dot * sampling_time;

    % --- Data Logging ---
    % Save the current state, control input, and time to their history logs.
    state_vector_history = [state_vector_history, states];
    control_input_history = [control_input_history; u_star];
    time_history = [time_history; time];

    % Update Simulation Time.
    time = time + sampling_time;
end
toc; % Stop the timer and display the elapsed time.

%% Plotting
% This section visualizes the simulation results.

% --- Animated Trajectory Plot ---
% This loop creates an animation of the robot's path from start to goal.
figure;
for plt_idx = 1:length(time_history)
    clf; % Clear the current figure.
    hold on;
    % Plot the start and goal points.
    plot(state_vector_history(1, 1), state_vector_history(2, 1), '.g', 'MarkerSize', 20);
    plot(goal_states(1, 1), goal_states(2, 1), '.r', 'MarkerSize', 20);
    % Plot the robot's path up to the current time step.
    plot(state_vector_history(1, 1:plt_idx), state_vector_history(2, 1:plt_idx), 'k', 'LineWidth', 3);
    % Plot the obstacle.
    plot(x_obstacle, y_obstacle, 'b', 'LineWidth', 3);
    hold off;
    axis('equal'); % Ensure aspect ratio is 1:1.
    xlabel('x (m)');
    ylabel('y (m)');
    legend('start', 'goal', 'robot path', 'obstacle', 'location', 'northwest');
    pause(0.01 * sampling_time); % Pause briefly to create animation effect.
end

% --- State History Plot ---
% This plot shows the evolution of each state variable (x, y, theta) over time.
figure;
subplot(3,1,1)
plot(time_history, state_vector_history(1, :), 'LineWidth', 3);
xlabel('t (s)'); ylabel('x (m)');
xlim([time_history(1), time_history(end)]);
subplot(3,1,2)
plot(time_history, state_vector_history(2, :), 'LineWidth', 3);
xlabel('t (s)'); ylabel('y (m)');
xlim([time_history(1), time_history(end)]);
subplot(3,1,3)
plot(time_history, state_vector_history(3, :), 'LineWidth', 3);
xlabel('t (s)'); ylabel('\theta (rad)');
xlim([time_history(1), time_history(end)]);
sgtitle('Robot State vs. Time');

% --- Control Input History Plot ---
% This plot shows the steering command issued by the controller over time.
figure;
plot(time_history(2:end), control_input_history, 'r', 'LineWidth', 3);
xlabel('t (s)'); ylabel('u (rad/s)');
xlim([time_history(1), time_history(end)]);
title('Control Input vs. Time');