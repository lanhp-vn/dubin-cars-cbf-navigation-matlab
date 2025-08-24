%% Two-Robot Collision Avoidance with a Dynamic Priority System
%
% This script simulates two Dubins car models navigating to their respective
% goals while avoiding collisions. The core of the simulation is a
% safety-critical controller using High-Order Control Barrier Functions (HOCBF)
% to guarantee inter-robot safety.
%
% A key feature of this script is a dynamic priority system. The robot
% closer to its goal is given "priority," which is enforced by a weighted
% cost function in the Quadratic Program (QP) controller. This makes the
% non-priority robot more likely to yield, resolving potential deadlocks.

%% Workspace Initialization
% Clear the workspace of all variables, close all open figures, and clear
% the command window to ensure a clean simulation environment.
clc;
clear;
close all;

%% Initialization
% This section defines all the core parameters for the simulation, robots,
% and controllers.

% --- Simulation & Robot Configuration ---
number_of_robots = 2; % The number of robots in the simulation.
time = 0; % Initialize simulation time to zero.
sampling_time = 1e-1; % Set the discrete time step for the simulation (s).

% --- State and Goal Definition ---
% 'states' defines the initial conditions [x-pos; y-pos; heading] for each robot.
states = [1, 2, 0; 0, 8, 0]';
% 'goal_states' defines the target state for each corresponding robot.
goal_states = [10, 8, 0; 10, 2, 0]';

% --- Robot Kinematics and Control ---
linear_velocity = 1; % The constant forward speed for the robots (m/s).
max_min_control_input = 3; % Symmetric control bounds for the steering input (rad/s).

% --- Controller Parameters ---
class_K_const = 1; % A positive constant (gain) for the CBF constraint.
safe_distance = 1; % The minimum allowable distance between robots (m).
max_simulation_time = 50; % A timeout to prevent the simulation from running indefinitely.

%% Priority Determination
% This section implements a simple heuristic to assign priority to one robot,
% which helps in resolving symmetric conflicts where both robots might
% otherwise hesitate or get stuck.

% Determine which robot is initially closer to its goal.
dist_to_goal_1 = norm(states(1:2, 1) - goal_states(1:2, 1));
dist_to_goal_2 = norm(states(1:2, 2) - goal_states(1:2, 2));

% The robot with the shorter distance to its goal is assigned priority.
if dist_to_goal_1 <= dist_to_goal_2
    priority_robot = 1;
else
    priority_robot = 2;
end

% --- Priority Logic Variables ---
priority_robot_reached_goal = false; % Flag to track if the priority robot has finished.
priority_weight = 100; % A high weight to penalize the priority robot's deviation from its nominal path.
normal_weight = 1;     % The standard weight for the non-priority robot.

%% Preallocation
% Define history vectors to log data for plotting and analysis.
state_vector_1_history = [];
control_input_1_history = [];
state_vector_2_history = [];
control_input_2_history = [];
time_history = [];

%% Simulate Trajectory
% This is the main simulation loop where robot states are updated over time.

% Initialize history logs with the starting conditions.
state_vector_1_history = [state_vector_1_history, states(:, 1)];
state_vector_2_history = [state_vector_2_history, states(:, 2)];
time_history = [time_history; time];
tic; % Start a timer to measure the simulation's execution time.

% The loop continues until all robots are near their goals or the simulation times out.
while norm(states(1:2, :) - goal_states(1:2, :)) >= 5e-2 && time < max_simulation_time
    
    % --- Dynamic Priority Logic ---
    % Check if the priority robot has reached its goal. Once it has, the
    % priority is released, and both robots are treated equally.
    if ~priority_robot_reached_goal && norm(states(1:2, priority_robot) - goal_states(1:2, priority_robot)) < 2e-1
        priority_robot_reached_goal = true;
        fprintf('Priority robot %d has reached its goal. Releasing priority.\n', priority_robot);
    end
    
    % Set the weights for the QP cost function based on the priority status.
    if priority_robot_reached_goal
        % If priority is released, treat both robots equally.
        w1 = normal_weight;
        w2 = normal_weight;
    else
        % Assign a high weight to the priority robot to make its path "more important"
        % in the QP optimization, encouraging the other robot to yield.
        if priority_robot == 1
            w1 = priority_weight;
            w2 = normal_weight;
        else
            w1 = normal_weight;
            w2 = priority_weight;
        end
    end
    
    % --- Nominal Controller (Goal-Seeking Behavior) ---
    % Computes a desired steering angle to point each robot towards its goal.
    kappa = 5;
    e_theta_1 = goal_states(1:2, 1) - states(1:2, 1);
    nominal_controller_1 = kappa * (atan2(e_theta_1(2), e_theta_1(1)) - states(3, 1));
    e_theta_2 = goal_states(1:2, 2) - states(1:2, 2);
    nominal_controller_2 = kappa * (atan2(e_theta_2(2), e_theta_2(1)) - states(3, 2));
    nominal_controller = [nominal_controller_1; nominal_controller_2];
    
    % --- Emergency Stop & System Dynamics ---
    % Dynamic velocity adjustment for safety.
    dist_between_robots = norm(states(1:2, 1) - states(1:2, 2));
    if dist_between_robots < 0.5
        current_velocity = 0;
    else
        current_velocity = linear_velocity;
    end
    % Define the Dubins car dynamics for each robot.
    f_1 = [current_velocity*cos(states(3, 1)), current_velocity*sin(states(3, 1)), 0]';
    g_1 = [0, 0, 1]';
    f_2 = [current_velocity*cos(states(3, 2)), current_velocity*sin(states(3, 2)), 0]';
    g_2 = [0, 0, 1]';
    
    % --- Combined Dynamics for Lie Derivative Calculations ---
    % Stack the state dynamics to correctly compute derivatives for the coupled system.
    f_x = [f_1; f_2];
    g_x_u1 = [g_1; zeros(3,1)]; % Shows the effect of control u1 on the combined state.
    g_x_u2 = [zeros(3,1); g_2]; % Shows the effect of control u2 on the combined state.
    
    % --- Safety-Critical Controller (HOCBF) ---
    % These calculations define the safety constraints for the QP.
    e_robot_position_1 = states(1:2, 1) - states(1:2, 2);
    robot_cbf_11 = norm(e_robot_position_1)^2 - safe_distance^2;
    robot_cbf_12 = 2 * (e_robot_position_1' * (current_velocity * [cos(states(3,1)); sin(states(3,1))] - current_velocity * [cos(states(3,2)); sin(states(3,2))]));
    robot_hocbf_1 = robot_cbf_12 + class_K_const*robot_cbf_11;
    % The gradient of the HOCBF w.r.t. the combined state vector [x1; y1; th1; x2; y2; th2].
    gradient_robot_hocbf_1 = [2*e_robot_position_1(1) + 2*current_velocity*cos(states(3, 1)), ...
                              2*e_robot_position_1(2) + 2*current_velocity*sin(states(3, 1)), ...
                              2*current_velocity*(-e_robot_position_1(1)*sin(states(3, 1)) + e_robot_position_1(2)*cos(states(3, 1))), ...
                             -2*e_robot_position_1(1) - 2*current_velocity*cos(states(3, 2)), ...
                             -2*e_robot_position_1(2) - 2*current_velocity*sin(states(3, 2)), ...
                              2*current_velocity*(e_robot_position_1(1)*sin(states(3, 2)) - e_robot_position_1(2)*cos(states(3, 2)))]';
    alpha_robot_hocbf_1 = class_K_const*robot_hocbf_1^3;
    
    % Repeat calculations for the second robot's perspective.
    e_robot_position_2 = -e_robot_position_1;
    robot_cbf_21 = robot_cbf_11;
    robot_cbf_22 = 2 * (e_robot_position_2' * (current_velocity * [cos(states(3,2)); sin(states(3,2))] - current_velocity * [cos(states(3,1)); sin(states(3,1))]));
    robot_hocbf_2 = robot_cbf_22 + class_K_const*robot_cbf_21;
    gradient_robot_hocbf_2 = [-2*e_robot_position_2(1) - 2*current_velocity*cos(states(3, 1)), ...
                              -2*e_robot_position_2(2) - 2*current_velocity*sin(states(3, 1)), ...
                               2*current_velocity*(e_robot_position_2(1)*sin(states(3, 1)) - e_robot_position_2(2)*cos(states(3, 1))), ...
                               2*e_robot_position_2(1) + 2*current_velocity*cos(states(3, 2)), ...
                               2*e_robot_position_2(2) + 2*current_velocity*sin(states(3, 2)), ...
                               2*current_velocity*(-e_robot_position_2(1)*sin(states(3, 2)) + e_robot_position_2(2)*cos(states(3, 2)))]';
    alpha_robot_hocbf_2 = class_K_const*robot_hocbf_2^3;
    
    % --- Corrected Lie Derivatives and QP Setup ---
    % Calculate the Lie derivatives using the combined system dynamics.
    L_f_H1 = gradient_robot_hocbf_1' * f_x;
    L_g1_H1 = gradient_robot_hocbf_1' * g_x_u1; % Effect of u1 on constraint 1.
    L_g2_H1 = gradient_robot_hocbf_1' * g_x_u2; % Effect of u2 on constraint 1.
    L_f_H2 = gradient_robot_hocbf_2' * f_x;
    L_g1_H2 = gradient_robot_hocbf_2' * g_x_u1; % Effect of u1 on constraint 2.
    L_g2_H2 = gradient_robot_hocbf_2' * g_x_u2; % Effect of u2 on constraint 2.
    
    % --- QP Formulation with Priority Weights ---
    % The objective is to find a control 'u' that minimizes the weighted
    % squared error from the nominal controller: min ||u - u_nominal||^2_W
    uhat = reshape(nominal_controller, number_of_robots, 1);
    W = diag([w1, w2]); % The diagonal weight matrix, implementing priority.
    H = 2 * W; % Quadratic term for the QP cost function.
    z = -2 * W * uhat; % Linear term for the QP cost function.
    
    % The safety constraints in the form A*u <= b.
    A = -[L_g1_H1, L_g2_H1; 
          L_g1_H2, L_g2_H2];
    b = [L_f_H1 + alpha_robot_hocbf_1; 
         L_f_H2 + alpha_robot_hocbf_2];
    
    opts = optimoptions(@quadprog, 'Display', 'off'); % Suppress solver output.
    % Solve the QP for the optimal safe control input.
    u_opt = quadprog(sparse(H), double(z), A, b, [], [], [], [], [], opts);
    
    % --- Input Saturation and State Integration ---
    for r_idx = 1:number_of_robots
        % If the QP is infeasible (no solution found), fallback to the nominal controller.
        if isempty(u_opt)
            u_star(r_idx) = nominal_controller(r_idx);
        else
            u_star(r_idx) = u_opt(r_idx);
        end
        % Enforce the physical control limits of the robot.
        if norm(u_star(r_idx), 'inf') > max_min_control_input
            u_star(r_idx) = max_min_control_input*sign(u_star(r_idx));
        end
        
        % Update the robot's state using Forward Euler integration if it hasn't reached its goal.
        if norm(states(1:2, r_idx) - goal_states(1:2, r_idx)) >= 5e-2
            if r_idx == 1
                x_dot = f_1 + g_1*u_star(r_idx);
            elseif r_idx == 2
                x_dot = f_2 + g_2*u_star(r_idx);
            end
        else
            x_dot = zeros(3, 1); % Stop the robot if it's at the goal.
        end
        states(:, r_idx) = states(:, r_idx) + x_dot*sampling_time;
    end
    
    % --- Data Logging ---
    state_vector_1_history = [state_vector_1_history, states(:, 1)];
    state_vector_2_history = [state_vector_2_history, states(:, 2)];
    control_input_1_history = [control_input_1_history, u_star(1)];
    control_input_2_history = [control_input_2_history, u_star(2)];
    time_history = [time_history; time];
    time = time + sampling_time;
end
toc; % Stop the timer and display elapsed time.

%% Plotting
% This section visualizes the simulation results, creating an animation of
% the robot trajectories and their safety zones.
figure;
for plt_idx = 1:10:length(time_history) % Speed up plotting by skipping frames.
    clf;
    hold on;
    grid on;
    
    % Plot start and goal points.
    plot(state_vector_1_history(1, 1), state_vector_1_history(2, 1), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(goal_states(1, 1), goal_states(2, 1), 'x', 'MarkerEdgeColor', 'r', 'MarkerSize', 12, 'LineWidth', 2);
    plot(state_vector_2_history(1, 1), state_vector_2_history(2, 1), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(goal_states(1, 2), goal_states(2, 2), 'x', 'MarkerEdgeColor', 'r', 'MarkerSize', 12, 'LineWidth', 2);
    % Plot robot paths up to the current time step.
    p5 = plot(state_vector_1_history(1, 1:plt_idx), state_vector_1_history(2, 1:plt_idx), 'm', 'LineWidth', 2);
    p6 = plot(state_vector_2_history(1, 1:plt_idx), state_vector_2_history(2, 1:plt_idx), 'k--', 'LineWidth', 2);
    % Plot current robot positions as squares.
    plot(state_vector_1_history(1, plt_idx), state_vector_1_history(2, plt_idx), 's', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', 'MarkerSize', 12);
    plot(state_vector_2_history(1, plt_idx), state_vector_2_history(2, plt_idx), 's', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'MarkerSize', 12);
    
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
    legend([p5, p6], 'Robot 1 Path', 'Robot 2 Path', 'location', 'northwest');
    drawnow; % Update the figure window.
    pause(0.01);
end
% Set the title for the final plot after the animation is complete.
title('Final Robot Trajectories');