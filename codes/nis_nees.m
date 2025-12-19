%% Run NEES/NIS Tests
% Aditya
%% --- NEES/NIS Calculation ---
NEES_all = NaN(num_step, N); % NEES for each step & MC run
NIS_all  = NaN(num_step, N); % NIS for each step & MC run
x_err_k = NaN(n,num_step,N);
y_err_k = NaN(p,num_step,N);

% --- Chi-squared bounds ---
alpha = 0.05; % 95% confidence
NEES_low  = chi2inv(alpha/2, N*n)/N;
NEES_high = chi2inv(1-alpha/2, N*n)/N;
NIS_low   = chi2inv(alpha/2, N*p)/N;
NIS_high  = chi2inv(1-alpha/2, N*p)/N;

good_NIS = 0;
good_NEES = 0;

for lv1 = 1:N
    for k = 1:num_step
        %% --- NEES ---
        x_err = x_true(:,k,lv1) - x_hat_k(:,k,lv1);               % Estimation error
        x_err(3) = wrapToPi(x_err(3));
        x_err(6) = wrapToPi(x_err(6));

        NEES_all(k,lv1) = x_err'/P_k(:,:,k,lv1)*x_err;

        %% --- NIS ---        
        y_err = y_true(:,k,lv1) - y_hat_k_m(:,k,lv1);              % Innovation
        y_err(1) = wrapToPi(y_err(1));
        y_err(3) = wrapToPi(y_err(3));

        NIS_all(k,lv1) = y_err'/S_k(:,:,k,lv1)*y_err;

        x_err_k(:,k,lv1) = x_err; % save values for debugging
        y_err_k(:,k,lv1) = y_err; % save values for debugging
    end
end

% --- Average over Monte Carlo runs ---
NEES_mean = mean(NEES_all,2);
NIS_mean  = mean(NIS_all,2);

for lv1 = 1:num_step
    if NEES_mean(lv1) > NEES_low && NEES_mean(lv1) < NEES_high
        good_NEES = good_NEES + 1;
    end
    if NIS_mean(lv1) > NIS_low && NIS_mean(lv1) < NIS_high
        good_NIS = good_NIS + 1;
    end
end

disp("NEES %: " + good_NEES*100/num_step);
disp("NIS %: " + good_NIS*100/num_step);


% %% --- NEES/NIS Plots ---
% figure;
% 
% % NEES Plot
% subplot(2,1,1);
% plot(t, NEES_mean, 'b.', 'LineWidth', 1); hold on;
% yline(NEES_low, 'r--', 'Lower 95% CI');
% yline(NEES_high, 'r--', 'Upper 95% CI');
% grid on;
% title('NEES - Filter Consistency');
% xlabel('Time [s]');
% ylabel('NEES');
% ylim([NEES_low-.5*NEES_high NEES_high*1.5])
% legend('Mean NEES','95% CI');
% 
% % NIS Plot
% subplot(2,1,2);
% plot(t, NIS_mean, 'b.', 'LineWidth', 1); hold on;
% yline(NIS_low, 'r--', 'Lower 95% CI');
% yline(NIS_high, 'r--', 'Upper 95% CI');
% grid on;
% title('NIS - Measurement Consistency');
% xlabel('Time [s]');
% ylabel('NIS');
% ylim([NIS_low-.5*NIS_high NIS_high*1.5])
% legend('Mean NIS','95% CI');

%% --- NEES/NIS Plots ---
figure;

% Font sizes
labelFS = 16;
titleFS = 18;
legendFS = 14;
tickFS  = 12;
ylineFS = 14; % font size for upper/lower line labels

%% --- NEES Plot ---
subplot(2,1,1);
plot(t, NEES_mean, 'b.', 'LineWidth', 1); hold on;

% NEES confidence lines
yl1 = yline(NEES_low, 'r--', 'Lower 95% CI', 'LineWidth', 1.5);
yl1.LabelVerticalAlignment = 'bottom';  % Lower label below line
yl1.FontSize = ylineFS;

yl2 = yline(NEES_high, 'r--', 'Upper 95% CI', 'LineWidth', 1.5);
yl2.FontSize = ylineFS;  % Upper label font size

grid on;
title('NEES - Filter Consistency', 'FontSize', titleFS);
xlabel('Time [s]', 'FontSize', labelFS);
ylabel('NEES', 'FontSize', labelFS);
set(gca, 'FontSize', tickFS);

ylim([NEES_low - 0.5*NEES_high, NEES_high*1.5]);
legend('Mean NEES','95% CI', 'FontSize', legendFS, 'Location','best');

%% --- NIS Plot ---
subplot(2,1,2);
plot(t, NIS_mean, 'b.', 'LineWidth', 1); hold on;

% NIS confidence lines
yl3 = yline(NIS_low, 'r--', 'Lower 95% CI', 'LineWidth', 1.5);
yl3.LabelVerticalAlignment = 'bottom';
yl3.FontSize = ylineFS;

yl4 = yline(NIS_high, 'r--', 'Upper 95% CI', 'LineWidth', 1.5);
yl4.FontSize = ylineFS;

grid on;
title('NIS - Measurement Consistency', 'FontSize', titleFS);
xlabel('Time [s]', 'FontSize', labelFS);
ylabel('NIS', 'FontSize', labelFS);
set(gca, 'FontSize', tickFS);

ylim([NIS_low - 0.5*NIS_high, NIS_high*1.5]);
legend('Mean NIS','95% CI', 'FontSize', legendFS, 'Location','best');
