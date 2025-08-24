clc; clear; close all;

%% Initialization
number_of_robots = 3;
time = 0;
sampling_time = 1e-1;
states = [1, 2, 0; 0, 8, 0; 2, 5, 0]';
goal_states = [5, 10, 0; 12, 3, 0; 18, 8, 0]';
linear_velocity = 1;
max_min_control_input = 3;
class_K_const = 1;
safe_distance = 5;
max_simulation_time = 100;

%% Priority Weights
weights = [100, 50, 1];  % For Priority 1, 2, 3

%% Preallocation
state_vector_1_history = []; control_input_1_history = [];
state_vector_2_history = []; control_input_2_history = [];
state_vector_3_history = []; control_input_3_history = [];
time_history = [];

%% Simulate Trajectory
state_vector_1_history = [state_vector_1_history, states(:, 1)];
state_vector_2_history = [state_vector_2_history, states(:, 2)];
state_vector_3_history = [state_vector_3_history, states(:, 3)];
time_history = [time_history; time];

tic
while any(vecnorm(states(1:2, :) - goal_states(1:2, :)) >= 5e-2) && time < max_simulation_time
    % Determine active robots (those not yet at their goals)
    reached_goal = vecnorm(states(1:2, :) - goal_states(1:2, :)) < 2e-1;
    active_robots = find(~reached_goal);
    
    if ~isempty(active_robots)
        % Calculate distances for active robots
        distances_active = vecnorm(states(1:2, active_robots) - goal_states(1:2, active_robots));
        [~, sorted_active] = sort(distances_active);
        priority_order_active = active_robots(sorted_active);
        
        % Assign weights based on priority
        w = ones(1, number_of_robots);  % Default for reached robots
        for i = 1:length(priority_order_active)
            w(priority_order_active(i)) = weights(i);
        end
        
        % Print current priority order
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
        w = ones(1, number_of_robots);  % All robots have reached goals
        fprintf('Time: %.2f s - All robots have reached their goals.\n', time);
    end

    kappa = 10;
    e_theta_1 = goal_states(1:2, 1) - states(1:2, 1);
    nominal_controller_1 = kappa * (atan2(e_theta_1(2), e_theta_1(1)) - states(3, 1));
    e_theta_2 = goal_states(1:2, 2) - states(1:2, 2);
    nominal_controller_2 = kappa * (atan2(e_theta_2(2), e_theta_2(1)) - states(3, 2));
    e_theta_3 = goal_states(1:2, 3) - states(1:2, 3);
    nominal_controller_3 = kappa * (atan2(e_theta_3(2), e_theta_3(1)) - states(3, 3));
    nominal_controller = [nominal_controller_1; nominal_controller_2; nominal_controller_3];

    dist_12 = norm(states(1:2, 1) - states(1:2, 2));
    dist_13 = norm(states(1:2, 1) - states(1:2, 3));
    dist_23 = norm(states(1:2, 2) - states(1:2, 3));
    if min([dist_12, dist_13, dist_23]) < 0.5
        current_velocity = 0;
    else
        current_velocity = linear_velocity;
    end

    f_1 = [current_velocity*cos(states(3, 1)), current_velocity*sin(states(3, 1)), 0]';
    g_1 = [0, 0, 1]';
    f_2 = [current_velocity*cos(states(3, 2)), current_velocity*sin(states(3, 2)), 0]';
    g_2 = [0, 0, 1]';
    f_3 = [current_velocity*cos(states(3, 3)), current_velocity*sin(states(3, 3)), 0]';
    g_3 = [0, 0, 1]';
    f_x = [f_1; f_2; f_3];
    g_x_u1 = [g_1; zeros(3,1); zeros(3,1)];
    g_x_u2 = [zeros(3,1); g_2; zeros(3,1)];
    g_x_u3 = [zeros(3,1); zeros(3,1); g_3];

    % HOCBF Constraints
    e_12 = states(1:2, 1) - states(1:2, 2);
    h_12 = norm(e_12)^2 - safe_distance^2;
    dh_12 = 2 * e_12' * (current_velocity * [cos(states(3,1)); sin(states(3,1))] - current_velocity * [cos(states(3,2)); sin(states(3,2))]);
    psi_12 = dh_12 + class_K_const * h_12;
    grad_psi_12 = [2*e_12(1) + 2*current_velocity*cos(states(3,1)), 2*e_12(2) + 2*current_velocity*sin(states(3,1)), ...
                   2*current_velocity*(-e_12(1)*sin(states(3,1)) + e_12(2)*cos(states(3,1))), ...
                   -2*e_12(1) - 2*current_velocity*cos(states(3,2)), -2*e_12(2) - 2*current_velocity*sin(states(3,2)), ...
                   2*current_velocity*(e_12(1)*sin(states(3,2)) - e_12(2)*cos(states(3,2))), zeros(1,3)]';
    alpha_12 = class_K_const * psi_12^3;

    e_13 = states(1:2, 1) - states(1:2, 3);
    h_13 = norm(e_13)^2 - safe_distance^2;
    dh_13 = 2 * e_13' * (current_velocity * [cos(states(3,1)); sin(states(3,1))] - current_velocity * [cos(states(3,3)); sin(states(3,3))]);
    psi_13 = dh_13 + class_K_const * h_13;
    grad_psi_13 = [2*e_13(1) + 2*current_velocity*cos(states(3,1)), 2*e_13(2) + 2*current_velocity*sin(states(3,1)), ...
                   2*current_velocity*(-e_13(1)*sin(states(3,1)) + e_13(2)*cos(states(3,1))), zeros(1,3), ...
                   -2*e_13(1) - 2*current_velocity*cos(states(3,3)), -2*e_13(2) - 2*current_velocity*sin(states(3,3)), ...
                   2*current_velocity*(e_13(1)*sin(states(3,3)) - e_13(2)*cos(states(3,3)))]';
    alpha_13 = class_K_const * psi_13^3;

    e_23 = states(1:2, 2) - states(1:2, 3);
    h_23 = norm(e_23)^2 - safe_distance^2;
    dh_23 = 2 * e_23' * (current_velocity * [cos(states(3,2)); sin(states(3,2))] - current_velocity * [cos(states(3,3)); sin(states(3,3))]);
    psi_23 = dh_23 + class_K_const * h_23;
    grad_psi_23 = [zeros(1,3), 2*e_23(1) + 2*current_velocity*cos(states(3,2)), 2*e_23(2) + 2*current_velocity*sin(states(3,2)), ...
                   2*current_velocity*(-e_23(1)*sin(states(3,2)) + e_23(2)*cos(states(3,2))), ...
                   -2*e_23(1) - 2*current_velocity*cos(states(3,3)), -2*e_23(2) - 2*current_velocity*sin(states(3,3)), ...
                   2*current_velocity*(e_23(1)*sin(states(3,3)) - e_23(2)*cos(states(3,3)))]';
    alpha_23 = class_K_const * psi_23^3;

    % QP Setup
    W = diag(w);
    H = 2 * W;
    z = -2 * W * nominal_controller;

    L_f_12 = grad_psi_12' * f_x;
    L_g1_12 = grad_psi_12' * g_x_u1;
    L_g2_12 = grad_psi_12' * g_x_u2;
    L_g3_12 = grad_psi_12' * g_x_u3;

    L_f_13 = grad_psi_13' * f_x;
    L_g1_13 = grad_psi_13' * g_x_u1;
    L_g2_13 = grad_psi_13' * g_x_u2;
    L_g3_13 = grad_psi_13' * g_x_u3;

    L_f_23 = grad_psi_23' * f_x;
    L_g1_23 = grad_psi_23' * g_x_u1;
    L_g2_23 = grad_psi_23' * g_x_u2;
    L_g3_23 = grad_psi_23' * g_x_u3;

    A = -[L_g1_12, L_g2_12, L_g3_12; L_g1_13, L_g2_13, L_g3_13; L_g1_23, L_g2_23, L_g3_23];
    b = [L_f_12 + alpha_12; L_f_13 + alpha_13; L_f_23 + alpha_23];

    opts = optimoptions(@quadprog, 'Display', 'off');
    u_opt = quadprog(sparse(H), double(z), A, b, [], [], [], [], [], opts);

    % Control Application
    for r_idx = 1:number_of_robots
        if isempty(u_opt)
            u_star(r_idx) = nominal_controller(r_idx);
        else
            u_star(r_idx) = u_opt(r_idx);
        end
        if norm(u_star(r_idx), 'inf') > max_min_control_input
            u_star(r_idx) = max_min_control_input * sign(u_star(r_idx));
        end
        if norm(states(1:2, r_idx) - goal_states(1:2, r_idx)) >= 5e-2
            if r_idx == 1
                x_dot = f_1 + g_1 * u_star(r_idx);
            elseif r_idx == 2
                x_dot = f_2 + g_2 * u_star(r_idx);
            else
                x_dot = f_3 + g_3 * u_star(r_idx);
            end
        else
            x_dot = zeros(3, 1);
        end
        states(:, r_idx) = states(:, r_idx) + x_dot * sampling_time;
    end

    state_vector_1_history = [state_vector_1_history, states(:, 1)];
    state_vector_2_history = [state_vector_2_history, states(:, 2)];
    state_vector_3_history = [state_vector_3_history, states(:, 3)];
    control_input_1_history = [control_input_1_history, u_star(1)];
    control_input_2_history = [control_input_2_history, u_star(2)];
    control_input_3_history = [control_input_3_history, u_star(3)];
    time_history = [time_history; time];
    time = time + sampling_time;
end
toc

%% Plotting
figure;
for plt_idx = 1:10:length(time_history)
    clf; hold on; grid on;
    plot(state_vector_1_history(1, 1), state_vector_1_history(2, 1), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(goal_states(1, 1), goal_states(2, 1), 'x', 'MarkerEdgeColor', 'r', 'MarkerSize', 12, 'LineWidth', 2);
    plot(state_vector_2_history(1, 1), state_vector_2_history(2, 1), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(goal_states(1, 2), goal_states(2, 2), 'x', 'MarkerEdgeColor', 'r', 'MarkerSize', 12, 'LineWidth', 2);
    plot(state_vector_3_history(1, 1), state_vector_3_history(2, 1), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    plot(goal_states(1, 3), goal_states(2, 3), 'x', 'MarkerEdgeColor', 'r', 'MarkerSize', 12, 'LineWidth', 2);

    p5 = plot(state_vector_1_history(1, 1:plt_idx), state_vector_1_history(2, 1:plt_idx), 'm', 'LineWidth', 2);
    p6 = plot(state_vector_2_history(1, 1:plt_idx), state_vector_2_history(2, 1:plt_idx), 'k--', 'LineWidth', 2);
    p7 = plot(state_vector_3_history(1, 1:plt_idx), state_vector_3_history(2, 1:plt_idx), 'c-.', 'LineWidth', 2);

    plot(state_vector_1_history(1, plt_idx), state_vector_1_history(2, plt_idx), 's', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', 'MarkerSize', 12);
    plot(state_vector_2_history(1, plt_idx), state_vector_2_history(2, plt_idx), 's', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'MarkerSize', 12);
    plot(state_vector_3_history(1, plt_idx), state_vector_3_history(2, plt_idx), 's', 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', 'MarkerSize', 12);

    th = 0:pi/50:2*pi;
    xunit1 = safe_distance * cos(th) + state_vector_1_history(1, plt_idx);
    yunit1 = safe_distance * sin(th) + state_vector_1_history(2, plt_idx);
    plot(xunit1, yunit1, 'b:', 'LineWidth', 1);

    % --- Visualize the Safety Distance ---
    % Draw a circle around the first robot to represent its safety bubble.
    th = 0:pi/50:2*pi;
    xunit1 = safe_distance * cos(th) + state_vector_1_history(1, plt_idx);
    yunit1 = safe_distance * sin(th) + state_vector_1_history(2, plt_idx);
    plot(xunit1, yunit1, 'b:', 'LineWidth', 1);

    hold off; axis('equal');
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('Robot Trajectories at Time: %.2f s', time_history(plt_idx)));
    legend([p5, p6, p7], 'Robot 1 Path', 'Robot 2 Path', 'Robot 3 Path', 'location', 'northwest');
    drawnow; pause(0.01);
end
title('Final Robot Trajectories');