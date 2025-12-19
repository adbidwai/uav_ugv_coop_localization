%% Calculate nominal state parameters
omega_g_nom = v_g/L*tan(phi_g); % UGV rotation rate (rad/s)
R_g_nom = abs(v_g/omega_g_nom); % UGV turning radius (m)
R_a_nom = abs(v_a/omega_a); % UAV turning radius (m)
xi_g_nom_c = xi_g_nom_0 + R_g_nom; % UGV radius center east position (m)
xi_a_nom_c = xi_a_nom_0 + R_a_nom; % UAV radius center east position (m)

%% Calculate nominal state history
theta_g_nom = theta_g_nom_0 + omega_g_nom*t; % UGV direction history (rad)
theta_a_nom = theta_a_nom_0 + omega_a*t; % UAV direction history (rad)

xi_g_nom = xi_g_nom_c - R_g_nom.*sin(theta_g_nom); % UGV nominal east position history (m)
eta_g_nom = R_g_nom.*cos(theta_g_nom); % UGV nominal north position history (m)

xi_a_nom = xi_a_nom_c + R_a_nom.*sin(theta_a_nom); % UAV nominal east position history (m)
eta_a_nom = -R_a_nom.*cos(theta_a_nom); % UAV nominal north position history (m)

x_nom = [xi_g_nom; eta_g_nom; theta_g_nom; xi_a_nom; eta_a_nom; theta_a_nom]; % Full nominal trajectory
u_nom = ones(1,num_step).*[v_g; phi_g; v_a; omega_a]; % Nominal input trajectory