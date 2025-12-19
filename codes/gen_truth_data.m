% Initialize variables
x_true = zeros(n,num_step,N); 
y_true = NaN(p,num_step,N); 

% Run N Monte Carlo nonlinear simulations with process noise
for lv1 = 1:N
    % Initialize variables
    w_k = mvnrnd(zeros(n,1), Qtrue, num_step); % Define process noise for every time step
    w_k = w_k'; % Turn into column vectors

    v_k = mvnrnd(zeros(p,1), Rtrue, num_step); % Define measurement noise for every time step
    v_k = v_k'; % Turn into column vectors
    
    %% Compute state
    % Compute state for k = 0,1 time steps with process noise
    x_true(:,1,lv1) = x_nom(:,1) + w_k(:,1); % Time step 0 
    
    % Compute state for k = 1,2,3... time step with process noise
    for lv2 = 2:num_step
        x_true(:,lv2,lv1) = f_km1(x_true(:,lv2-1,lv1), u_nom(:,lv2-1), w_k(:,lv2), const);
        y_true(:,lv2,lv1) = h_k(x_true(:,lv2,lv1),v_k(:,lv2));
        
        %% Angle Wrapping
        y_true(1,lv2,lv1) = wrapToPi(y_true(1,lv2,lv1));
        y_true(3,lv2,lv1) = wrapToPi(y_true(3,lv2,lv1));

        x_true(3,lv2,lv1) = wrapToPi(x_true(3,lv2,lv1));
        x_true(6,lv2,lv1) = wrapToPi(x_true(6,lv2,lv1));
    end
end