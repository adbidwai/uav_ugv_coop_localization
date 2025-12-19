function line_plot(dim, x, y, x_label, y_label, xlimits, ylimits, titles, line_style, line_width, color, plot_size, grid_type)
%% Description
% Function that plots y vs x
% INPUTS
%   dim : Dimension of subplots [rows cols] (Int)
%   x : X values for all plots left-right top-bottom [x1; x2; ...] (Double)
%   y : Y values for all plots left-right top-bottom [y1; y2; ...] (Double)
%   x_label : X axes labels for all plots left-right top-bottom [x1_label; x2_label; ...] (String)
%   y_label : Y axes labels for all plots left-right top-bottom [y1_label; y2_label; ...] (String)
%   titles : Titles for all plots left-right top-bottom [title1; title2; ...] (String)
%   line_style : Line styles '-', '--', ':', '-.', 'none' (String)
%   line_width : Line width 0.5 default (Double)
%   color : RGB triplet, hexadecimal code
%   plot_size : Position/size of plot, four element vector [0.13 0.11 0.775 0.815] default
%   grid_type : Grid setting for all plots, 'on', 'off', 'minor' (String)
%
% Michael States
% Updated Dec 2025

%% Check validity of inputs
% ill do this later

%% Plot
m = dim(1); % Number of rows
n = dim(2); % Number of columns

figure
for lv1 = 1:m % Rows
    for lv2 = 1:n % Cols
        p = (lv1-1)*n + lv2; % Subplot index

        subplot(dim(1),dim(2),p)
        plot(x(p,:),y(p,:),'LineStyle',line_style,'LineWidth',line_width,'Color',color)
        xlabel(x_label(p),'Interpreter','latex')
        ylabel(y_label(p),'Interpreter','latex')
        if ~isempty(xlimits)
            xlim(xlimits(p,:))
        end
        if ~isempty(ylimits)
            ylim(ylimits(p,:))
        end
        title(titles(p),'Interpreter','latex')
        grid(grid_type)
        set(gca,'TickLabelInterpreter','latex')
        hold on
    end
end
set(gcf,"Position",plot_size)