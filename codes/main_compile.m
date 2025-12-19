clear;
close all;
clc;

% rng(14) % debugging

%% Set parameters
% Load values from file
load("cooplocalization_finalproj_KFdata.mat")

% Rtrue = Rtrue.*0.01; % testing/debugging
% Qtrue = Qtrue.*0.01; % testing/debugging

% Set values
const_struct

% UGV parameters
L = const.L; % UGV Wheelbase (m)
xi_g_nom_0 = const.xi_g_nom_0; % UGV initial East position STATE (m)
eta_g_nom_0 = const.eta_g_nom_0; % UGV initial North position STATE (m)
theta_g_nom_0 = const.theta_g_nom_0; % UGV initial direction STATE (rad)
v_g = const.v_g; % UGV velocity INPUT (m/s)
phi_g = const.phi_g; % UGV turning angle INPUT (rad)

% UAV parameters
xi_a_nom_0 = const.xi_a_nom_0; % UAV initial East position STATE (m)
eta_a_nom_0 = const.eta_a_nom_0; % UAV initial North position STATE (m)
theta_a_nom_0 = const.theta_a_nom_0; % UAV initial direction STATE (rad)
v_a = const.v_a; % UAV velocity INPUT (m/s)
omega_a = const.omega_a; % UAV rotation rate INPUT (rad/s)

% General Parameters
t0 = const.t0; % Initial Time (s)
tf = const.tf; % Final time (s)
Dt = const.Dt; % Time step (s)
t = const.t; % Time steps (s)
num_step = const.num_step; % Number of steps (including 0)
n = const.n; % Number of states
m = const.m; % Number of inputs
p = const.p; % Number of outputs
N = const.N; % Number of Monte Carlo simulations
Qtune = const.Qtune; % Process noise covariance for filter Tuning knob for EKF
Rtune = const.Rtune; % Measurement noise covariance

% Monte Carlo Parameters
x_hat_0 = const.x_hat_0; % Initial state estimate
P_0_p = const.P_0_p; % Initial error covariance estimate

%% Calculate nominal parameters
calc_nom_traj

%% Generate truth data with process noise
tic
gen_truth_data
disp("Ground truth data generated")
toc

%% Run generated data through Extended Kalman Filter
x_hat_k = NaN(n,num_step,N);
P_k = NaN(n,n,num_step,N);
S_k = NaN(p,p,num_step,N);
y_hat_k_m = NaN(p,num_step,N);

for lv1 = 1:N
    [x_hat_k(:,:,lv1), P_k(:,:,:,lv1), S_k(:,:,:,lv1), y_hat_k_m(:,:,lv1)] = EKF(x_hat_0, P_0_p, u_nom(:,1), y_true(:,:,lv1), Qtune, Rtune, const);
end
disp("EKF Finished")
toc

%% Run NEES/NIS Tests
nis_nees

%% Plot Results
x_plot1 = [x_hat_k(1,:,1); x_hat_k(4,:,1); x_true(1,:,1); x_true(4,:,1)];
y_plot1 = [x_hat_k(2,:,1); x_hat_k(5,:,1); x_true(2,:,1); x_true(5,:,1)];
x_labels1 = ["East Pos (m)" "East Pos (m)" "East Pos (m)" "East Pos (m)"];
y_labels1 = ["North Pos (m)" "North Pos (m)" "North Pos (m)" "North Pos (m)"];
xlimits1 = [6 20; -100 200; 6 20; -100 200];
ylimits1 = [-7 7; -150 150; -7 7; -150 150];
titles1 = ["UGV Pos Est History" "UAV Pos Est History" "UGV Pos History" "UAV Pos History"];
plot_size1 = [150 150 600 600];
line_plot([2 2],x_plot1,y_plot1,x_labels1,y_labels1,xlimits1,ylimits1,titles1,'-',2,'r',plot_size1,'minor')

x_plot2 = [t; t; t; t];
y_plot2 = [x_hat_k(3,:,1); x_hat_k(6,:,1); x_true(3,:,1); x_true(6,:,1)];
x_labels2 = ["Time (s)" "Time (s)" "Time (s)" "Time (s)"];
y_labels2 = ["Direction (rad)" "Direction (rad)" "Direction (rad)" "Direction (rad)"];
xlimits2 = [];
ylimits2 = [];
titles2 = ["UGV Dir Est History" "UAV Dir Est History" "UGV Dir History" "UAV Dir History"];
plot_size2 = [150 150 600 600];
line_plot([2 2],x_plot2,y_plot2,x_labels2,y_labels2,xlimits2,ylimits2,titles2,'-',2,'r',plot_size2,'minor')

x_plot3 = [t; t];
x_labels3 = ["Time (s)" "Time (s)"];
y_labels3 = ["UGV East Pos Est Error (m)" "UAV East Pos Est Error (m)"];
xlimits3 = [];
ylimits3 = [];
titles3 = ["UGV East Pos Est Error History" "UAV East Pos Est Error History"];
plot_size3 = [150 150 800 600];


% figure
% for lv1 = 1:2 % Rows
%     for lv2 = 1:3 % Cols
%         p = (lv1-1)*3 + lv2; % Subplort index
% 
%         subplot(2,3,p)
%         plot(t,x_err_k(p,:,1),'.','LineWidth',2,'Color','b')
%         % xlabel(x_labels3(p),'Interpreter','latex')
%         % ylabel(y_labels3(p),'Interpreter','latex')
%         % title(titles3(p),'Interpreter','latex')
%         grid minor
%         set(gca,'TickLabelInterpreter','latex')
%         hold on
%         plot(t,2*squeeze(P_k(p,p,:,1))'.^2,'LineStyle','--','LineWidth',2,'Color','r')
%         plot(t,-2*squeeze(P_k(p,p,:,1))'.^2,'LineStyle','--','LineWidth',2,'Color','r')
%     end
% end
% set(gcf,"Position",plot_size3)

%% Plot 4: EKF State Estimation Errors with ±2σ Bounds
figure

state_names = { ...
    'UGV East Position', ...
    'UGV North Position', ...
    'UGV Heading', ...
    'UAV East Position', ...
    'UAV North Position', ...
    'UAV Heading'};

for lv1 = 1:2 % Rows
    for lv2 = 1:3 % Cols
        p = (lv1-1)*3 + lv2; % Subplot index

        subplot(2,3,p)

        % Estimation error
        h1 = plot(t, x_err_k(p,:,1), '.', ...
            'LineWidth', 1.5, 'Color', 'b');
        hold on

        % ±2σ covariance bounds (unchanged functionality)
        h2 = plot(t,  2*squeeze(P_k(p,p,:,1))'.^(1/2), '--', ...
            'LineWidth', 1.5, 'Color', 'r');
        h3 = plot(t, -2*squeeze(P_k(p,p,:,1))'.^(1/2), '--', ...
            'LineWidth', 1.5, 'Color', 'r');

        % Labels and titles
        xlabel('Time (s)', 'Interpreter', 'latex')
        ylabel('Error', 'Interpreter', 'latex')
        title(state_names{p}, 'Interpreter', 'latex')

        grid minor
        set(gca, 'TickLabelInterpreter', 'latex')
    end
end

% -------- Global Legend --------
lgd = legend([h1 h2 h3], ...
    {'Estimation Error', '$+2\sigma$', '$-2\sigma$'}, ...
    'Interpreter', 'latex', ...
    'Orientation', 'horizontal');

lgd.Orientation = 'vertical';
lgd.Position = [0.01 0.35 0.15 0.3];   % left-center


% -------- Global Title --------
sgtitle('EKF State Estimation Errors with $\pm2\sigma$ Bounds', ...
    'Interpreter', 'latex')

% -------- Figure Size --------
set(gcf, 'Position', plot_size3)




x_plot1 = [x_hat_k(1,:,2); x_hat_k(4,:,2); x_true(1,:,2); x_true(4,:,2)];
y_plot1 = [x_hat_k(2,:,2); x_hat_k(5,:,2); x_true(2,:,2); x_true(5,:,2)];
x_labels1 = ["East Pos (m)" "East Pos (m)" "East Pos (m)" "East Pos (m)"];
y_labels1 = ["North Pos (m)" "North Pos (m)" "North Pos (m)" "North Pos (m)"];
xlimits1 = [6 20; -100 200; 6 20; -100 200];
ylimits1 = [-7 7; -150 150; -7 7; -150 150];
titles1 = ["UGV Pos Est History" "UAV Pos Est History" "UGV Pos History" "UAV Pos History"];
plot_size1 = [150 150 600 600];
line_plot([2 2],x_plot1,y_plot1,x_labels1,y_labels1,xlimits1,ylimits1,titles1,'-',2,'r',plot_size1,'minor')


x_plot2 = [t; t; t; t];
y_plot2 = [x_hat_k(3,:,2); x_hat_k(6,:,2); x_true(3,:,2); x_true(6,:,2)];
x_labels2 = ["Time (s)" "Time (s)" "Time (s)" "Time (s)"];
y_labels2 = ["Direction (rad)" "Direction (rad)" "Direction (rad)" "Direction (rad)"];
xlimits2 = [];
ylimits2 = [];
titles2 = ["UGV Dir Est History" "UAV Dir Est History" "UGV Dir History" "UAV Dir History"];
plot_size2 = [150 150 600 600];
line_plot([2 2],x_plot2,y_plot2,x_labels2,y_labels2,xlimits2,ylimits2,titles2,'-',2,'r',plot_size2,'minor')
