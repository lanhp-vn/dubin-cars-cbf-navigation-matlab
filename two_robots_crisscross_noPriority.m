%% Two-Robot Collision Avoidance Simulation using HOCBF
%
% This script simulates the navigation of two Dubins car models, each moving
% towards a respective goal while actively avoiding collisions with each other.
% The control strategy is based on a safety-critical framework that uses
% High-Order Control Barrier Functions (HOCBF) formulated as a
% Quadratic Program (QP). An additional safety layer is included to
% dynamically reduce velocity if robots get critically close.

%% Workspace Initialization
% Clear the workspace of all variables, close all open figures, and clear
% the command window to ensure a clean simulation environment.
clc;
clear;
close all;

%% Initialization
% This section defines the core parameters for the simulation, including
% the number of robots, their initial/goal states, and controller settings.

% --- Simulation & Robot Configuration ---
number_of_robots = 2; % The number of robots participating in the simulation.
time = 0; % Initialize the simulation time to zero.
sampling_time = 1e-1; % Set the discrete time step for the simulation (s).

% --- State and Goal Definition ---
% 'states' defines the initial conditions for each robot. Each row is a
% robot, and columns are [x-pos, y-pos, heading]. The transpose (') makes
% each robot a column vector.
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

%% Preallocation
% Define history vectors to log data for plotting and analysis. While this
% script dynamically grows arrays, preallocation is a good practice for
% performance in larger simulations.
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

% The loop continues until all robots are within a small threshold of their
% goals or the maximum simulation time is exceeded.
while norm(states(1:2, :) - goal_states(1:2, :)) >= 5e-2 && time < max_simulation_time
    % --- Nominal Controller (Goal-Seeking Behavior) ---
    % This controller computes a desired steering angle to point each robot
    % towards its respective goal using a proportional control law.
    kappa = 10; % Proportional gain for the nominal controllers.
    e_theta_1 = goal_states(1:2, 1) - states(1:2, 1);
    nominal_controller_1 = kappa * (atan2(e_theta_1(2), e_theta_1(1)) - states(3, 1));
    e_theta_2 = goal_states(1:2, 2) - states(1:2, 2);
    nominal_controller_2 = kappa * (atan2(e_theta_2(2), e_theta_2(1)) - states(3, 2));
    nominal_controller = [nominal_controller_1; nominal_controller_2];

    % --- Dynamic Velocity Adjustment for Emergency Safety ---
    % This is an additional safety layer. If robots get critically close,
    % their velocity is set to zero to prevent collision.
    dist_between_robots = norm(states(1:2, 1) - states(1:2, 2));
    if dist_between_robots < 0.5 % A critical distance threshold.
        current_velocity = 0; % Stop the robots for safety.
    else
        current_velocity = linear_velocity;
    end

    % --- System Dynamics (Dubins Car Model) ---
    % Defines how each robot's state changes. 'f' is the drift term
    % (movement without control), and 'g' maps the control input to the
    % state derivatives.
    f_1 = [current_velocity * cos(states(3, 1)), current_velocity * sin(states(3, 1)), 0]';
    g_1 = [0, 0, 1]';
    f_2 = [current_velocity * cos(states(3, 2)), current_velocity * sin(states(3, 2)), 0]';
    g_2 = [0, 0, 1]';

    % --- Safety-Critical Controller (HOCBF for Collision Avoidance) ---
    % This controller modifies the nominal control to guarantee inter-robot
    % safety. A High-Order Control Barrier Function (HOCBF) is used because
    % the control input (steering) does not directly affect the robot's
    % position (relative degree is 2).

    % -- HOCBF Constraint for Robot 1 with respect to Robot 2 --
    e_robot_position_1 = states(1:2, 1) - states(1:2, 2);
    robot_cbf_11 = norm(e_robot_position_1)^2 - safe_distance^2; % Base CBF (h)
    robot_cbf_12 = 2 * current_velocity * (e_robot_position_1(1) * cos(states(3, 1)) + e_robot_position_1(2) * sin(states(3, 1))); % Derivative of h (L_f h)
    robot_hocbf_1 = robot_cbf_12 + class_K_const * robot_cbf_11; % The HOCBF (psi)
    % Gradient of the HOCBF with respect to the full state vector [states_1; states_2].
    gradient_robot_hocbf_1 = [2 * e_robot_position_1(1) + 2 * current_velocity * cos(states(3, 1)), ...
                              2 * e_robot_position_1(2) + 2 * current_velocity * sin(states(3, 1)), ...
                              2 * current_velocity * (-e_robot_position_1(1) * sin(states(3, 2)) + e_robot_position_1(2) * cos(states(3, 1))), ...
                              -2 * e_robot_position_1(1) - 2 * current_velocity * cos(states(3, 1)), ...
                              -2 * e_robot_position_1(2) - 2 * current_velocity * sin(states(3, 1)), 0]';
    % Lie derivatives needed for the final QP constraint.
    lie_derivitive_f_robot_hocbf_1 = gradient_robot_hocbf_1' * [f_1; f_2];
    lie_derivitive_g_robot_hocbf_1 = gradient_robot_hocbf_1' * [g_1; g_2];
    alpha_robot_hocbf_1 = class_K_const * robot_hocbf_1^3; % Extended class K function.

    % -- HOCBF Constraint for Robot 2 with respect to Robot 1 (Symmetric) --
    e_robot_position_2 = states(1:2, 2) - states(1:2, 1);
    robot_cbf_21 = norm(e_robot_position_2)^2 - safe_distance^2;
    robot_cbf_22 = 2 * current_velocity * (e_robot_position_2(1) * cos(states(3, 2)) + e_robot_position_2(2) * sin(states(3, 2)));
    robot_hocbf_2 = robot_cbf_22 + class_K_const * robot_cbf_21;
    gradient_robot_hocbf_2 = [2 * e_robot_position_2(1) + 2 * current_velocity * cos(states(3, 2)), ...
                              2 * e_robot_position_2(2) + 2 * current_velocity * sin(states(3, 2)), ...
                              2 * current_velocity * (-e_robot_position_2(1) * sin(states(3, 2)) + e_robot_position_2(2) * cos(states(3, 2))), ...
                              -2 * e_robot_position_2(1) - 2 * current_velocity * cos(states(3, 2)), ...
                              -2 * e_robot_position_2(2) - 2 * current_velocity * sin(states(3, 2)), 0]';
    lie_derivitive_f_robot_hocbf_2 = gradient_robot_hocbf_2' * [f_2; f_1];
    lie_derivitive_g_robot_hocbf_2 = gradient_robot_hocbf_2' * [g_2; g_1];
    alpha_robot_hocbf_2 = class_K_const * robot_hocbf_2^3;

    % --- Quadratic Program (QP) for Optimal Safe Control ---
    % The QP finds a control input 'u' that is as close as possible to the
    % nominal controller's input while satisfying the HOCBF safety constraints.
    uhat = reshape(nominal_controller, 1 * number_of_robots, 1);
    H = 2 * eye(1 * number_of_robots); % Quadratic cost matrix to minimize ||u - u_nominal||^2.
    z = -2 * uhat; % Linear cost vector.
    % The safety constraints in the form A*u <= b.
    A = [-lie_derivitive_g_robot_hocbf_1 0; 0 -lie_derivitive_g_robot_hocbf_2];
    b = [alpha_robot_hocbf_1 + lie_derivitive_f_robot_hocbf_1; alpha_robot_hocbf_2 + lie_derivitive_f_robot_hocbf_2];
    opts = optimoptions(@quadprog, 'Display', 'off'); % Suppress QP solver output.
    % Solve the QP to find the optimal safe control input vector.
    u_opt = quadprog(sparse(H), double(z), A, b, [], [], [], [], [], opts);

    % --- State Update and Saturation ---
    % This loop applies the calculated control input to update each robot's state.
    for r_idx = 1:number_of_robots
        % Saturate the control input to respect the robot's physical limits.
        if norm(u_opt(r_idx), 'inf') > max_min_control_input
            u_star(r_idx) = max_min_control_input * u_opt(r_idx) / norm(u_opt(r_idx));
        else
            u_star(r_idx) = u_opt(r_idx);
        end
        % Only update the state if the robot has not yet reached its goal.
        if norm(states(1:2, r_idx) - goal_states(1:2, r_idx)) >= 5e-2
            if r_idx == 1
                x_dot = f_1 + g_1 * u_star(r_idx);
            elseif r_idx == 2
                x_dot = f_2 + g_2 * u_star(r_idx);
            end
        else
            x_dot = zeros(3, 1); % Stop the robot if it has reached its goal.
        end
        % Update the robot's state using Forward Euler integration.
        states(:, r_idx) = states(:, r_idx) + x_dot * sampling_time;
    end

    % --- Data Logging ---
    % Save the current state, control input, and time to their history logs.
    state_vector_1_history = [state_vector_1_history, states(:, 1)];
    state_vector_2_history = [state_vector_2_history, states(:, 2)];
    control_input_1_history = [control_input_1_history, u_star(1)];
    control_input_2_history = [control_input_2_history, u_star(2)];
    time_history = [time_history; time];
    % Update Simulation Time.
    time = time + sampling_time;
end
toc; % Stop the timer and display the elapsed simulation time.

%% Plotting
% This section visualizes the simulation results by creating an animation of
% the robot trajectories.
figure;
for plt_idx = 1:10:length(time_history)
    clf; % Clear the current figure to draw the next frame.
    hold on;
    % Plot the start and goal points for both robots.
    p1 = plot(state_vector_1_history(1, 1), state_vector_1_history(2, 1), '.g', 'MarkerSize', 20);
    p2 = plot(goal_states(1, 1), goal_states(2, 1), '.r', 'MarkerSize', 20);
    p3 = plot(state_vector_2_history(1, 1), state_vector_2_history(2, 1), '.g', 'MarkerSize', 20);
    p4 = plot(goal_states(1, 2), goal_states(2, 2), '.r', 'MarkerSize', 20);
    % Plot the trajectory paths up to the current time step.
    p5 = plot(state_vector_1_history(1, 1:plt_idx), state_vector_1_history(2, 1:plt_idx), 'm', 'LineWidth', 3);
    p6 = plot(state_vector_2_history(1, 1:plt_idx), state_vector_2_history(2, 1:plt_idx), 'k', 'LineWidth', 3);
    hold off;
    axis('equal'); % Ensure aspect ratio is 1:1 for a true-to-life visualization.
    xlabel('x (m)');
    ylabel('y (m)');
    legend([p1, p2, p5, p6], 'start', 'goal', 'robot 1 path', 'robot 2 path', 'location', 'northwest');
    pause(0.01 * sampling_time); % Pause briefly to create the animation effect.
end