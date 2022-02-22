%% Tracking Response Plotter Script

clear
clc
close all

%% Import the proportional data from spreadsheet
[date, min, sec, ms, samples, x_des, x_act, P_des, P_act, k_p, k_i]...
    = readvars('2-22-22\50_90_prop.xlsx');

%% Import the PI data from the spreadsheet
[date_i, min_i, sec_i, ms_i, samples_i, x_des_i, x_act_i, P_des_i, P_act_i, k_p_i, k_i_i]...
    = readvars('2-22-22\50_90_int.xlsx');

%% Lower and upper bounds for the plots
lower_bd = 50;
upper_bd = 90;

%% Develop plotting time vectors
time_plot = time_devel_func(sec, ms);
time_plot_i = time_devel_func(sec_i, ms_i);

%% Plot the reference and tracking signals
plotter_func(time_plot, x_des, x_act, time_plot_i, x_des_i, x_act_i, lower_bd, upper_bd);

%% time vector development function
function [time_new] = time_devel_func(sec, ms)

% Concatenate the second and millisecond vectors
time = sec + ms/1000;

% Loop to find the time differences between different samples
time_diff = zeros(1,length(sec)-1)';         % stores the time difference
for i = 1:(length(sec) - 1)
    if (sec(i+1) - sec(i)) < 0               % checks for 59 to 0 second situations
        time_diff(i) = (60 - time(i)) + time(i+1);
    else
        time_diff(i) = time(i+1) - time(i);
    end
end

% Loop to develop time vector
time_new = zeros(1, length(sec))';
for i = 2:length(sec)
    time_new(i) = time_new(i-1) + time_diff(i-1);
end

end

%% Plotting function for reference and tracking signals
function plotter_func(time_plot, x_des, x_act, time_plot_i, x_des_i, x_act_i, low_bd, up_bd)
figure

% Proportional signal plot (reference and actual positions)
subplot(2, 1, 1);
plot(time_plot,x_des,'LineWidth',1);        % reference position
hold on 
plot(time_plot,x_act,'LineWidth',1);        % actual position
legend('Reference','Actual', 'Location', 'southeast');
ylim([low_bd-5, up_bd+10]);
xlim([0, time_plot(end)+0.5]);
ylabel('x position [mm]');
xlabel('time [sec]');
title("P Signal Response (" + low_bd + " mm to " + up_bd + " mm)");

% PI signal plot (reference and actual positions)
subplot(2, 1, 2);
plot(time_plot_i,x_des_i,'LineWidth',1);    % reference position
hold on
plot(time_plot_i,x_act_i,'LineWidth',1);    % actual position
legend('Reference', 'Actual', 'Location', 'southeast');
ylim([low_bd-5, up_bd+10]);
xlim([0, time_plot_i(end)+0.5]);
ylabel('x position [mm]');
xlabel('time [sec]');
title("PI Signal Response (" + low_bd + " mm to " + up_bd + " mm)");
end