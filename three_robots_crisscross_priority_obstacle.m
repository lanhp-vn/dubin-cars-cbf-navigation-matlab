%% Three-Robot Collision Avoidance with a Static Obstacle and Dynamic Priority
%
% This script simulates three Dubins car models navigating to their respective
% goals while avoiding collisions with each other and a fixed circular obstacle.
% A dynamic priority system assigns higher priority to robots closer to their
% goals, enforced via a weighted QP cost function. Safety is maintained using
% Higher-Order Control Barrier Functions (HOCBFs).

%% Workspace Initialization
% Clear the workspace of all variables, close all open figures, and clear
% the command window to ensure a clean simulation environment.
clc;
clear;
close all;

%% Initialization
% This section defines all the core parameters for the simulation, including
% robot and environment properties, and controller settings.

% --- Simulation & Robot Configuration ---
number_of_robots = 3; % The number of robots participating in the simulation.
time = 0; % Initialize the simulation time to zero.
sampling_time = 1e-1; % Set the discrete time step for the simulation (s).

% --- State and Goal Definition (criss-crossing scenario) ---
% 'states' defines initial conditions [x-pos; y-pos; heading] for each robot.
states = [0, 2, 0; 0, 8, 0; 2, 5, 0]';
% 'goal_states' defines the target state for each corresponding robot.
goal_states = [5, 10, 0; 12, 3, 0; 18, 8, 0]';

% --- Robot Kinematics and Control ---
linear_velocity = 1; % The constant forward speed for the robots (m/s).
max_min_control_input = 3; % Symmetric control bounds for the steering input (rad/s).

% --- Environment Definition ---
obstacle_position = [5, 5]'; % Center position of the circular obstacle [x; y] (m).
obstacle_radius = 1.5; % Radius of the circular obstacle (m).
% Generate points for plotting the circular obstacle.
x_obstacle = obstacle_position(1) + obstacle_radius*cos(linspace(0, 2*pi));
y_obstacle = obstacle_position(2) + obstacle_radius*sin(linspace(0, 2*pi));

% --- Controller Parameters ---
class_K_const = 1; % A positive constant (gain) for the CBF constraints.
safe_distance = 3.0; % The minimum allowable distance between any pair of robots (m).
max_simulation_time = 50; % A timeout to prevent the simulation from running indefinitely.

%% Preallocation
% Define history vectors to log data for plotting and analysis.
state_vector_1_history = [];
control_input_1_history = [];
state_vector_2_history = [];
control_input_2_history = [];
state_vector_3_history = [];
control_input_3_history = [];
time_history = [];

%% Simulate Trajectory
% This is the main simulation loop where robot states are updated over time.

% Initialize history logs with the starting conditions.
state_vector_1_history = [state_vector_1_history, states(:, 1)];
state_vector_2_history = [state_vector_2_history, states(:, 2)];
state_vector_3_history = [state_vector_3_history, states(:, 3)];
time_history = [time_history; time];

tic; % Start a timer to measure the simulation's execution time.

% The loop continues until all robots are within a small threshold of their
% goals or the maximum simulation time is exceeded.
while norm(states(1:2, :) - goal_states(1:2, :)) >= 5e-2 && time < max_simulation_time
    
    % --- Dynamic Priority Determination ---
    % At each time step, this section re-evaluates which robots are still
    % active and ranks them based on their proximity to their goals.
    weights = [100, 50, 1]; % Predefined weights for Priority 1, 2, and 3.
    % Determine which robots have not yet reached their goal.
    reached_goal = vecnorm(states(1:2, :) - goal_states(1:2, :)) < 2e-1;
    active_robots = find(~reached_goal); % Get indices of active robots.
    
    if ~isempty(active_robots)
        % Calculate distances to the goal ONLY for the active robots.
        distances_active = vecnorm(states(1:2, active_robots) - goal_states(1:2, active_robots));
        [~, sorted_active] = sort(distances_active); % Sort them by distance.
        % Create an ordered list of robot indices from highest to lowest priority.
        priority_order_active = active_robots(sorted_active);
        
        % Assign weights for the QP based on the current priority ranking.
        w = ones(1, number_of_robots); % Default weight for robots that have reached their goal.
        for i = 1:length(priority_order_active)
            % Assign the predefined priority weights to the active robots in rank order.
            w(priority_order_active(i)) = weights(i);
        end
        
        % Print the current priority order to the command window for real-time monitoring.
        if length(priority_order_active) == 3
            fprintf('Time: %.2f s - Priority Order: Robot %d (P1), Robot %d (P2), Robot %d (P3)\n', ...
                    time, priority_order_active(1), priority_order_active(2), priority_order_active(3));
        elseif length(priority_order_active) == 2
            fprintf('Time: %.2f s - Priority Order: Robot %d (P1), Robot %d (P2)\n', ...
                    time, priority_order_active(1), priority_order_active(2));
        elseif length(priority_order_active) == 1
            fprintf('Time: %.2f s - Priority Order: Robot %d (P1)\n', ...
                    time, priority_order_active(1));
        end
    else
        % This case handles the final time step when all robots have arrived.
        w = ones(1, number_of_robots);
        fprintf('Time: %.2f s - All robots have reached their goals.\n', time);
    end
    
    % --- Nominal Controller (Goal-Seeking Behavior) ---
    % Computes a desired steering angle for each robot to point towards its goal.
    kappa = 10;
    e_theta_1 = goal_states(1:2, 1) - states(1:2, 1);
    nominal_controller_1 = kappa * (atan2(e_theta_1(2), e_theta_1(1)) - states(3, 1));
    e_theta_2 = goal_states(1:2, 2) - states(1:2, 2);
    nominal_controller_2 = kappa * (atan2(e_theta_2(2), e_theta_2(1)) - states(3, 2));
    e_theta_3 = goal_states(1:2, 3) - states(1:2, 3);
    nominal_controller_3 = kappa * (atan2(e_theta_3(2), e_theta_3(1)) - states(3, 3));
    nominal_controller = [nominal_controller_1; nominal_controller_2; nominal_controller_3];
    
    % --- System Dynamics ---
    % Defines how each robot's state changes based on the Dubins car model.
    f_1 = [linear_velocity*cos(states(3, 1)), linear_velocity*sin(states(3, 1)), 0]';
    g_1 = [0, 0, 1]';
    f_2 = [linear_velocity*cos(states(3, 2)), linear_velocity*sin(states(3, 2)), 0]';
    g_2 = [0, 0, 1]';
    f_3 = [linear_velocity*cos(states(3, 3)), linear_velocity*sin(states(3, 3)), 0]';
    g_3 = [0, 0, 1]';
    
    %% Safety-Critical Controller (HOCBF Constraints)
    % This section defines all safety constraints that will be enforced by the QP.
    % It includes 3 robot-obstacle constraints and 3 inter-robot constraints.
    
    % -- Obstacle avoidance constraints for each robot --
    % Robot 1 vs Obstacle
    e_obs_1 = states(1:2, 1) - obstacle_position;
    h_obs_1 = norm(e_obs_1)^2 - obstacle_radius^2;
    L_f_h_obs_1 = 2 * e_obs_1' * (f_1(1:2));
    B_obs_1 = L_f_h_obs_1 + class_K_const * h_obs_1; % HOCBF
    L_f_B_obs_1 = 2 * f_1(1:2)' * f_1(1:2) + class_K_const * L_f_h_obs_1;
    L_g_B_obs_1 = 2 * e_obs_1' * [-linear_velocity*sin(states(3,1)); linear_velocity*cos(states(3,1))];
    b1 = L_f_B_obs_1 + class_K_const * B_obs_1; % RHS of the QP constraint
    % Robot 2 vs Obstacle
    e_obs_2 = states(1:2, 2) - obstacle_position;
    h_obs_2 = norm(e_obs_2)^2 - obstacle_radius^2;
    L_f_h_obs_2 = 2 * e_obs_2' * (f_2(1:2));
    B_obs_2 = L_f_h_obs_2 + class_K_const * h_obs_2;
    L_f_B_obs_2 = 2 * f_2(1:2)' * f_2(1:2) + class_K_const * L_f_h_obs_2;
    L_g_B_obs_2 = 2 * e_obs_2' * [-linear_velocity*sin(states(3,2)); linear_velocity*cos(states(3,2))];
    b2 = L_f_B_obs_2 + class_K_const * B_obs_2;
    % Robot 3 vs Obstacle
    e_obs_3 = states(1:2, 3) - obstacle_position;
    h_obs_3 = norm(e_obs_3)^2 - obstacle_radius^2;
    L_f_h_obs_3 = 2 * e_obs_3' * (f_3(1:2));
    B_obs_3 = L_f_h_obs_3 + class_K_const * h_obs_3;
    L_f_B_obs_3 = 2 * f_3(1:2)' * f_3(1:2) + class_K_const * L_f_h_obs_3;
    L_g_B_obs_3 = 2 * e_obs_3' * [-linear_velocity*sin(states(3,3)); linear_velocity*cos(states(3,3))];
    b3 = L_f_B_obs_3 + class_K_const * B_obs_3;
    
    % -- Inter-robot collision avoidance constraints for each pair --
    % Robot 1 vs Robot 2
    e_r1_r2 = states(1:2, 1) - states(1:2, 2);
    h_r12 = norm(e_r1_r2)^2 - safe_distance^2;
    L_f_h_r12 = 2 * e_r1_r2' * (f_1(1:2) - f_2(1:2));
    L_g1_h_r12 = 2 * e_r1_r2' * [-linear_velocity*sin(states(3,1)); linear_velocity*cos(states(3,1))];
    L_g2_h_r12 = -2 * e_r1_r2' * [-linear_velocity*sin(states(3,2)); linear_velocity*cos(states(3,2))];
    b4 = L_f_h_r12 + class_K_const * h_r12;
    % Robot 1 vs Robot 3
    e_r1_r3 = states(1:2, 1) - states(1:2, 3);
    h_r13 = norm(e_r1_r3)^2 - safe_distance^2;
    L_f_h_r13 = 2 * e_r1_r3' * (f_1(1:2) - f_3(1:2));
    L_g1_h_r13 = 2 * e_r1_r3' * [-linear_velocity*sin(states(3,1)); linear_velocity*cos(states(3,1))];
    L_g3_h_r13 = -2 * e_r1_r3' * [-linear_velocity*sin(states(3,3)); linear_velocity*cos(states(3,3))];
    b5 = L_f_h_r13 + class_K_const * h_r13;
    % Robot 2 vs Robot 3
    e_r2_r3 = states(1:2, 2) - states(1:2, 3);
    h_r23 = norm(e_r2_r3)^2 - safe_distance^2;
    L_f_h_r23 = 2 * e_r2_r3' * (f_2(1:2) - f_3(1:2));
    L_g2_h_r23 = 2 * e_r2_r3' * [-linear_velocity*sin(states(3,2)); linear_velocity*cos(states(3,2))];
    L_g3_h_r23 = -2 * e_r2_r3' * [-linear_velocity*sin(states(3,3)); linear_velocity*cos(states(3,3))];
    b6 = L_f_h_r23 + class_K_const * h_r23;
    
    %% QP Setup
    % The QP finds an optimal control 'u' that minimizes the weighted
    % deviation from the nominal controller, subject to all safety constraints.
    W = diag(w); % The diagonal weight matrix, implementing the dynamic priority.
    H = 2 * W;   % Quadratic term for the QP cost function: min ||u - u_nominal||^2_W
    z = -2 * W * nominal_controller; % Linear term for the QP cost function.
    
    % Assemble the constraint matrix A and vector b for the QP (A*u <= b).
    A = [-L_g_B_obs_1, 0, 0;             % Row 1: Robot 1 vs Obstacle
         0, -L_g_B_obs_2, 0;             % Row 2: Robot 2 vs Obstacle
         0, 0, -L_g_B_obs_3;             % Row 3: Robot 3 vs Obstacle
         -L_g1_h_r12, -L_g2_h_r12, 0;    % Row 4: Robot 1 vs Robot 2
         -L_g1_h_r13, 0, -L_g3_h_r13;    % Row 5: Robot 1 vs Robot 3
         0, -L_g2_h_r23, -L_g3_h_r23];   % Row 6: Robot 2 vs Robot 3
    b = [b1; b2; b3; b4; b5; b6];
    
    opts = optimoptions(@quadprog, 'Display', 'off'); % Suppress solver output.
    % Solve for the optimal safe control input.
    u_opt = quadprog(sparse(H), double(z), A, b, [], [], [], [], [], opts);
    
    %% Determine and Saturate Control Inputs
    % This section ensures that the calculated control is valid and within
    % the physical limits of the robots.
    u_star = zeros(number_of_robots, 1);
    if isempty(u_opt)
        % If the QP is infeasible (no solution found), fallback to the nominal controller.
        u_star = nominal_controller;
        fprintf('QP infeasible at time %.2f, using nominal control.\n', time);
    else
        u_star = u_opt;
    end
    % Saturate the final control inputs to respect physical limits.
    for r_idx = 1:number_of_robots
        if norm(u_star(r_idx), 'inf') > max_min_control_input
            u_star(r_idx) = max_min_control_input * sign(u_star(r_idx));
        end
    end
    
    %% Update States
    % This loop applies the calculated control inputs to update the state of each robot.
    for r_idx = 1:number_of_robots
        % Update state using Forward Euler integration if the robot hasn't reached its goal.
        if norm(states(1:2, r_idx) - goal_states(1:2, r_idx)) >= 5e-2
            if r_idx == 1
                x_dot = f_1 + g_1 * u_star(1);
            elseif r_idx == 2
                x_dot = f_2 + g_2 * u_star(2);
            else
                x_dot = f_3 + g_3 * u_star(3);
            end
        else
            x_dot = zeros(3, 1); % Stop the robot if it's at the goal.
        end
        states(:, r_idx) = states(:, r_idx) + x_dot * sampling_time;
    end
    
    %% History Logging
    % Save the current state, control input, and time to their history logs.
    state_vector_1_history = [state_vector_1_history, states(:, 1)];
    state_vector_2_history = [state_vector_2_history, states(:, 2)];
    state_vector_3_history = [state_vector_3_history, states(:, 3)];
    control_input_1_history = [control_input_1_history; u_star(1)];
    control_input_2_history = [control_input_2_history; u_star(2)];
    control_input_3_history = [control_input_3_history; u_star(3)];
    time_history = [time_history; time];
    time = time + sampling_time;
end
toc; % Stop the timer and display elapsed time.

%% Final Announcement
% Check if the simulation ended because the goals were reached or it timed out.
if time < max_simulation_time
    fprintf('\nMission Complete! All robots have reached their goals.\n');
    fprintf('Total simulation time: %.2f seconds.\n', time);
else
    fprintf('\nSimulation timed out before all goals were reached.\n');
    fprintf('Total simulation time: %.2f seconds.\n', time);
end

%% Plotting
% This section visualizes the simulation results, creating an animation of
% the robot and obstacle trajectories.
figure;
for plt_idx = 1:10:length(time_history) % Speed up plotting by skipping frames
    clf; hold on; grid on;
    % Plot the static obstacle.
    p_obs = plot(x_obstacle, y_obstacle, 'r-', 'LineWidth', 2);
    % Plot start and goal points for all robots.
    plot(state_vector_1_history(1, 1), state_vector_1_history(2, 1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal_states(1, 1), goal_states(2, 1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(state_vector_2_history(1, 1), state_vector_2_history(2, 1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal_states(1, 2), goal_states(2, 2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(state_vector_3_history(1, 1), state_vector_3_history(2, 1), 'co', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal_states(1, 3), goal_states(2, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    % Plot robot paths up to the current time step.
    p1 = plot(state_vector_1_history(1, 1:plt_idx), state_vector_1_history(2, 1:plt_idx), 'g-', 'LineWidth', 2);
    p2 = plot(state_vector_2_history(1, 1:plt_idx), state_vector_2_history(2, 1:plt_idx), 'b--', 'LineWidth', 2);
    p3 = plot(state_vector_3_history(1, 1:plt_idx), state_vector_3_history(2, 1:plt_idx), 'c-.', 'LineWidth', 2);
    % Plot current robot positions as squares.
    plot(state_vector_1_history(1, plt_idx), state_vector_1_history(2, plt_idx), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(state_vector_2_history(1, plt_idx), state_vector_2_history(2, plt_idx), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    plot(state_vector_3_history(1, plt_idx), state_vector_3_history(2, plt_idx), 'cs', 'MarkerFaceColor', 'c', 'MarkerSize', 10);

    % --- Visualize the Safety Distance ---
    % Draw a circle around the first robot to represent its safety bubble.
    th = 0:pi/50:2*pi;
    xunit1 = safe_distance * cos(th) + state_vector_1_history(1, plt_idx);
    yunit1 = safe_distance * sin(th) + state_vector_1_history(2, plt_idx);
    plot(xunit1, yunit1, 'b:', 'LineWidth', 1);
    
    hold off;
    axis('equal'); % Ensure a 1:1 aspect ratio.
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('Robot Trajectories at Time: %.2f s', time_history(plt_idx)));
    legend([p1, p2, p3, p_obs], 'Robot 1', 'Robot 2', 'Robot 3', 'Obstacle', 'location', 'best');
    drawnow; % Update the figure window.
    pause(0.01);
end
% Set the title for the final plot after the animation is complete.
title('Final Robot Trajectories');