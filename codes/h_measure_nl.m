function Y = h_measure_nl(x)
xi_g = x(1);
eta_g = x(2);
theta_g = x(3);
xi_a = x(4);
eta_a = x(5);
theta_a = x(6);
delta_xi = xi_a - xi_g;
delta_eta = eta_a - eta_g;
Y1 = atan2(delta_eta,delta_xi) - theta_g;
Y2 = sqrt(delta_xi^2 + delta_eta^2);
Y3 = atan2(delta_eta,delta_xi) - theta_a;
Y4 = xi_a;
Y5 = eta_a;
Y = [Y1; Y2; Y3; Y4; Y5];
end