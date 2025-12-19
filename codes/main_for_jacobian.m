clear;
close all;
clc;

%% CHECKING JACOBIANS FOR PART 1
% % Define symbolic variables
% syms xi_g eta_g theta_g xi_a eta_a theta_a...
%     v_g phi_g v_a omega_a...
%     w_xg w_yg w_wg w_xa w_ya w_wa...
%     L;
% 
% % Define state
% x = [xi_g; eta_g; theta_g; xi_a; eta_a; theta_a];
% 
% % Define inputs
% u = [v_g; phi_g; v_a; omega_a];
% 
% % Define noise terms
% w = [w_xg; w_yg; w_wg; w_xa; w_ya; w_wa];
% 
% % Define outputs
% y = [atan((eta_a - eta_g)/(xi_a - xi_g)) - theta_g;...
%     sqrt((xi_g - xi_a)^2 + (eta_g - eta_a)^2);...
%     atan((eta_g - eta_a)/(xi_g - xi_a)) - theta_a;...
%     xi_a;...
%     eta_a];
% 
% % Define CT dynamics
% f = [v_g*cos(theta_g); v_g*sin(theta_g); v_g/L*tan(phi_g); ...
%     v_a*cos(theta_a); v_a*sin(theta_a); omega_a];
% 
% % Compute Jacobians
% df_dx = jacobian(f, x);
% df_du = jacobian(f, u);
% dy_dx = jacobian(y, x);
% dy_du = jacobian(y, u);

%% Evaluate Jacobians at each nominal condition along the trajectory
t0 = 0; % Initial Time (s)
tf = 100; % Final time (s)
Dt = .1; % Time step (s)
t = t0:Dt:tf; % Time steps (s)
num_step = length(t); % # of steps (including 0)
n = 6; % Number of states
m = 4; % Number of inputs
p = 5; % Number of outputs

% Define UGV parameters
L = 0.5; % UGV wheelbase (m)
v_g = 2; % UGV velocity (m/s)
phi_g = -pi/18; % UGV turning angle (rad)
theta_g_0 = pi/2; % UGV initial direction (rad)
xi_g_0 = 10; % UGV initial east position (m)
eta_g_0 = 0; % UGV initial north position (m)

% Define UAV parameters
theta_a_0 = -pi/2; % UAV initial direction (rad)
v_a = 12; % UAV velocity (m/s)
omega_a = pi/25; % UAV rotation rate (rad/s)
xi_a_0 = -60; % UAV initial east position (m)
eta_a_0 = 0; % UAV initial north position (m)

% Calculate nominal state parameters
omega_g = v_g/L*tan(phi_g); % UGV rotation rate (rad/s)
R_g = abs(v_g/omega_g); % UGV turning radius (m)
R_a = abs(v_a/omega_a); % UAV turning radius (m)
xi_g_c = xi_g_0 + R_g; % UGV radius center east position (m)
xi_a_c = xi_a_0 + R_a; % UAV radius center east position (m)

% Calculate nominal state history
theta_g = theta_g_0 + omega_g*t; % UGV direction history (rad)
theta_a = theta_a_0 + omega_a*t; % UAV direction history (rad)

xi_g = xi_g_c - R_g.*sin(theta_g); % UGV nominal east position history (m)
eta_g = R_g.*cos(theta_g); % UGV nominal north position history (m)

xi_a = xi_a_c + R_a.*sin(theta_a); % UAV nominal east position history (m)
eta_a = -R_a.*cos(theta_a); % UAV nominal north position history (m)

% Calculate A(t), B(t), and C(t) to find F(t), G(t), and H(t) at each time step
for lv1 = 1:num_step
    xt = [xi_g(lv1); eta_g(lv1); theta_g(lv1); xi_a(lv1); eta_a(lv1); theta_a(lv1)]; % state at step lv1 (t)
    ut = [v_g; phi_g; v_a; omega_a]; % inputs at step lv1 (t) (constant)

    x_nom(:,lv1) = xt; % Calculate nominal state trajectory

    [At, Bt, Ct] = eval_jacobian(xt, ut, L); % Find A, B, and C at time step lv1 (t)
    At_hat = [At, Bt; zeros(m, n + m)]; % Generate augmented matrix
    exp_At_hat = expm(At_hat*Dt); % Compute matrix exponential

    y_nom(:,lv1) = Ct*xt;
    
    F(1:n, 1:n, lv1) = exp_At_hat(1:n,1:n); % Select F matrix
    G(1:n, 1:m, lv1) = exp_At_hat(1:n, n+1:end); % Select G matrix
    H(1:p, 1:n, lv1) = Ct; % Select H matrix
end

%% Test Simulation of DT Linearized Dynamics
% Define initial conditions
dx0 = [0; 1; 0; 0; 0; 0.1]; % Initial perturbations

% First step
dx = F(:,:,1)*dx0;

% 2+ step
for lv1 = 2:num_step
    dx(:,lv1) = F(:,:,lv1)*dx(:,lv1 - 1); % Compute state perturbation at next time step
    dy(:,lv1) = H(:,:,lv1)*dx(:,lv1); % Compute output perturbation at current time step
end

% Add nominal states/outputs and perturbations
x = x_nom + dx;
y = y_nom + dy;

%% Plot results
plot_size = [150 150 600 600];

figure
subplot(6,1,1)
plot(t, x(1,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time Step k",'Interpreter','latex')
ylabel("$\xi_g$ (m)",'Interpreter','latex')
title("Simulated Discrete Time Linearized State vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
ylim([8, 18])

subplot(6,1,2)
plot(t, x(2,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time Step k",'Interpreter','latex')
ylabel("$\eta_g$ (m)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized State vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
ylim([-4, 6])

subplot(6,1,3)
plot(t, x(3,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time Step k",'Interpreter','latex')
ylabel("$\theta_g$ (rad)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized State vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
% ylim([8, 18])

subplot(6,1,4)
plot(t, x(4,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time Step k",'Interpreter','latex')
ylabel("$\xi_a$ (m)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized State vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
ylim([-100, 140])

subplot(6,1,5)
plot(t, x(5,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time Step k",'Interpreter','latex')
ylabel("$\eta_a$ (m)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized State vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
ylim([-100, 140])

subplot(6,1,6)
plot(t, x(6,:),'LineStyle','-','LineWidth',2)
grid on;
xlabel("Time (s)",'Interpreter','latex')
ylabel("$\theta_a$ (rad)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized State vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
% ylim([8, 18])
saveas(gcf,"DT_LZ_state_vs_time.jpg");

% Plot outputs
figure
subplot(5,1,1)
plot(t,y(1,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time (s)",'Interpreter','latex')
ylabel("$y_1$ (rad)",'Interpreter','latex')
title("Simulated Discrete Time Linearized Output vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
% ylim([-1, 140])

subplot(5,1,2)
plot(t,y(2,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time (s)",'Interpreter','latex')
ylabel("$y_2$ (m)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized Output vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
% ylim([-1, 140])

subplot(5,1,3)
plot(t,y(3,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time (s)",'Interpreter','latex')
ylabel("$y_3$ (rad)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized Output vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
% ylim([-1, 140])

subplot(5,1,4)
plot(t,y(4,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time (s)",'Interpreter','latex')
ylabel("$y_4$ (m)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized Output vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
ylim([-100, 140])

subplot(5,1,5)
plot(t,y(5,:),'LineStyle','-','LineWidth',2)
grid on;
% xlabel("Time (s)",'Interpreter','latex')
ylabel("$y_5$ (m)",'Interpreter','latex')
% title("Simulated Discrete Time Linearized Output vs Time",'Interpreter','latex')
set(gcf,"Position",plot_size)
set(gca,'TickLabelInterpreter','latex')
ylim([-100, 140])
saveas(gcf,"DT_LZ_output_vs_time.jpg");