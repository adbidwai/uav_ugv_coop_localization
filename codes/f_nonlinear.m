function dxdt = f_nonlinear(t, x, u, L)
%state extraction
%state vector has ugv first and then uav
%ugv
xi_g = x(1);
eta_g = x(2);
theta_g = x(3);
%uav
xi_a = x(4);
eta_a = x(5);
theta_a = x(6);
%input extraction
%same as state, first ugv and then uav
v_g = u(1);
phi_g = u(2);
v_a = u(3);
w_a = u(4);
%dynamics defined as follows:
%ugv
xi_g_dot = v_g*cos(theta_g);
eta_g_dot = v_g*sin(theta_g);
theta_g_dot = (v_g/L)*tan(phi_g);
%uav
xi_a_dot = v_a*cos(theta_a);
eta_a_dot = v_a*sin(theta_a);
theta_a_dot = w_a;
dxdt = [xi_g_dot; eta_g_dot; theta_g_dot; xi_a_dot; eta_a_dot;
theta_a_dot];
end