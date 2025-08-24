% Dubins Car Simulation with Proportional Control and Dynamic Velocity
%
% This script simulates the trajectory of a Dubins car model, which is a
% simplified representation of a wheeled vehicle that can only move forward
% and turn at a bounded rate. The simulation implements a proportional
% controller for steering and two different methods for dynamic velocity
% adjustment to navigate from a start point to a goal.

%% Workspace Initialization
% Clear workspace of all variables, close all figures, and clear the command window.
clear;
close all;
clc;

%% Simulation Parameters
% --- Control Gains and Setpoints ---
v_max = 5;          % Maximum allowable forward velocity of the car (m/s).
v_min = 0.1;        % Minimum forward velocity to ensure the car is always moving (m/s).
k_v = 1;            % Proportional gain for the distance-based velocity control method.
kappa = 10;         % Proportional gain for the steering controller (rad/s).
goal = [50; 35];    % Target position vector [x_goal; y_goal] (m).

% --- State and Time Configuration ---
initial_state = [0; 0; 0]; % Initial state vector [x_pos; y_pos; heading_angle] (m, m, rad).
dt = 0.1;           % Time step for the simulation (s).
t_max = 20;         % Maximum duration of the simulation (s).
threshold = 0.05;   % Distance threshold to the goal at which the simulation stops (m).

% --- Control Method Selection ---
% Selects the velocity control strategy.
% Method 1: Velocity is proportional to the inverse of the heading error.
%           (i.e., slow down when turning sharply).
% Method 2: Velocity is proportional to the distance to the goal.
%           (i.e., slow down when approaching the target).
method = 1;

%% Variable Initialization
% Initialize the state vector with the initial conditions.
states = initial_state;
% Create a history log of the state vector for plotting the trajectory.
state_history = states;
% Create a history log for the velocity magnitude.
v_history = [];
% Create a history log for time.
time_history = 0;
% Initialize the simulation time counter.
t = 0;

%% Main Simulation Loop
% The loop runs until the maximum simulation time is reached or the car
% reaches the goal.
while t < t_max
    % Extract current state components for clarity.
    px = states(1);     % Current x-position
    py = states(2);     % Current y-position
    theta = states(3);  % Current heading angle

    % Calculate the Euclidean distance to the goal.
    d = sqrt((goal(1) - px)^2 + (goal(2) - py)^2);

    % Check for goal completion. If the car is within the threshold
    % distance, stop the car and exit the loop.
    if d < threshold
        v = 0; % Set final velocity to zero.
        break;
    end

    % --- Proportional Control Law ---
    % Calculate the desired heading angle towards the goal.
    theta_d = atan2(goal(2) - py, goal(1) - px);

    % Calculate the heading error. The result is wrapped to the [-pi, pi]
    % interval to ensure the shortest turn direction is chosen.
    e_theta = atan2(sin(theta_d - theta), cos(theta_d - theta));

    % Calculate the steering control input (angular velocity) using a
    % proportional controller.
    u1 = kappa * e_theta;

    % --- Dynamic Velocity Control ---
    % Adjust the forward velocity 'v' based on the selected method.
    if method == 1
        % Heading-based velocity: Slow down when the heading error is large.
        v = v_max * (1 - abs(e_theta)/pi);
        v = max(v, v_min); % Ensure velocity does not fall below the minimum.
    elseif method == 2
        % Distance-based velocity: Slow down as the car approaches the goal.
        v = min(k_v * d, v_max); % Velocity is proportional to distance, capped by v_max.
        v = max(v, v_min);       % Ensure velocity does not fall below the minimum.
    else
        error('Invalid method selected for velocity control.');
    end

    % --- State Update (Forward Euler Integration) ---
    % Update the car's position and heading based on the Dubins car model.
    px_next = px + v * cos(theta) * dt;
    py_next = py + v * sin(theta) * dt;
    theta_next = theta + u1 * dt;
    states = [px_next; py_next; theta_next]; % Assemble the new state vector.

    % --- History Logging ---
    % Append the new state, velocity, and time to their respective logs.
    state_history = [state_history, states];
    v_history = [v_history, v];
    time_history = [time_history, t + dt];

    % Increment simulation time.
    t = t + dt;
end

%% Plotting Results
% --- Trajectory Plot ---
figure;
plot(state_history(1, :), state_history(2, :), 'b-', 'LineWidth', 2);
hold on;
plot(initial_state(1), initial_state(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot(goal(1), goal(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
hold off;
axis equal; % Ensures the x and y axes have the same scale for a true-to-life path.
xlabel('x-position (m)');
ylabel('y-position (m)');
title('Dubins Car Trajectory');
legend('Path', 'Start', 'Target', 'Location', 'northwest');
grid on;

% --- Velocity vs. Time Plot ---
figure;
plot(time_history(2:end), v_history, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs. Time');
grid on;

% --- Distance to Target vs. Time Plot ---
% Calculate the distance to the goal at each time step.
distance_history = sqrt(sum((state_history(1:2, :) - goal).^2, 1));
figure;
plot(time_history, distance_history, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Distance to Target (m)');
title('Distance to Target vs. Time');
grid on;