%% Tracking Response Plotter Script
%{
Author: William Wang
Description: This script plots the reference and actual positions for the
one dimensional case of the soft robot
%}

clear
clc
close all

%% Find all the csv files in the directory and obtain the data
% specify directory of interest
files = dir('2-28-22\*.xlsx');

% get data from all the files in the directory
[time_plot, x_des, x_act] = get_data(files);

%% Plot the reference and tracked signals for the ramp input signal
% specify lower and upper bounds of the ramp
ramp_low_bd = [50, 50, 50];
ramp_up_bd = [90, 90, 90];

% specify the periods of the ramp
ramp_period = [120, 60, 20];

% plot the ramp signals
ramp_time = [time_plot(1), time_plot(3), time_plot(2)];         % reorganize ramp data
ramp_x_des = [x_des(1), x_des(3), x_des(2)];
ramp_x_act = [x_act(1), x_act(3), x_act(2)];

ramp_plotter_func(ramp_time, ramp_x_des, ramp_x_act, ramp_period, ramp_low_bd, ramp_up_bd);

%% Plot the reference and tracked signals for the sinusoid input signal
% specify upper and lower bounds of the sinusoid signal
sin_low_bd = [70 55 70];
sin_up_bd = [80 65 80];

% specify the frequencies of the input signal
sin_freq = [0.05, 0.075, 0.1];

% plot the sinusoid signals for f = 0.05, 0.075, 0.1
sinusoid_plotter_func(time_plot(4:6), x_des(4:6), x_act(4:6), sin_freq, sin_low_bd, sin_up_bd);

% plot the second set of sinusoid signals for f = 0.2, 0.5, 1
sin_low_bd = [70 70 70];
sin_freq = [0.2, 0.5, 1];
sin_up_bd = [80 80 80];

sinusoid_plotter_func(time_plot(7:9), x_des(7:9), x_act(7:9), sin_freq, sin_low_bd, sin_up_bd);


% < ===================================================================== >

%% The following are functions used in this script

%% time vector development function
function [time_new] = time_devel_func(sec, ms)
% Description: This function determines the time that has passed since data
% collection has started based on the seconds and milliseconds determined
% from the log file
% Inputs: the time stamps in seconds (sec) and milliseconds (ms)
% Output: the time vector for the elapsed time of the experiment (time_new)

% Concatenate the second and millisecond vectors
time = sec + ms/1000;

% Loop to find the time differences between different samples
time_diff = zeros(1,length(sec)-1)';         % stores the time difference between samples

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


%% Function to iterate over files to read information in each file
function [time_plot, x_des, x_act] = get_data(files)
% Description: This function reads all the data from a single directory and
% stores the data into cells
% Inputs: the files object that carries the information about the
% different data files in a specific directory (files
% Outputs: the time vector elapsed from the start of the experiment
% (time_plot), the actual and desired positions (x_act, x_des)

% Initialize cells to store the values from the experiments
date = {}; min = {}; sec = {}; millis = {}; samples = {}; x_des = {}; x_act = {}; P_des = {}; P_act = {}; k_prop = {}; k_int = {};

% loop through the files and store the respective values
for i=1:length(files)
    filename = strcat(files(i).folder, '\', files(i).name);
    [d, m, s, ms, samp, x_d, x_a, P_d, P_a, k_p, k_i]...
        = readvars(filename); 
    date(i) = {d};                  % date
    min(i) = {m};                   % minutes
    sec(i) = {s};                   % seconds
    millis(i) = {ms};               % milliseconds
    samples(i) = {samp};            % sample number
    x_des(i) = {x_d};               % desired positions
    x_act(i) = {x_a};               % actual positions
    P_des(i) = {P_d};               % desired pressures
    P_act(i) = {P_a};               % actual pressures
    k_prop(i) = {k_p};              % proportional gain
    k_int(i) = {k_i};               % integral gain
    time_plot(i) = {time_devel_func(s, ms)};    % obtain the time differences between samples
end

end

%% Plotting function for reference and tracking signals
function step_plotter_func(time_plot, x_des, x_act, time_plot_i, x_des_i, x_act_i, low_bd, up_bd)
% Description: This function plots the reference and actual step response
% of the system
% Inputs: the time vector (time_plot), the desired and actual positions of
% x for the P and PI controller respectively
% Outputs: none

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

%% Function to plot the ramp responses
function ramp_plotter_func(time_plot, x_des, x_act, period, low_bd, up_bd)
% Description: this function is used to plot the ramp signals into one 
% figure as series of subplots (note that the maximum subplots you should
% plot for a single figure is 3 for clarity). 
% Inputs: the time vector (time_plot - cell), the desired and actual positions
% (x_des, x_act - cell), the period of the input ramp (period - vector), the upper
% and lower bounds of the ramp signal (up_bd, low_bd - vector)
% Outputs: None

% Determine the number of plots by counting the number of cells
num_plots = length(time_plot);

figure

% loop to plot all the plots into a single figure
for i = 1:num_plots
    subplot(num_plots, 1, i);
    plot(time_plot{i},x_des{i},'LineWidth',1.5);        % reference position
    hold on 
    plot(time_plot{i},x_act{i},'LineWidth',1);        % actual position
    legend('Reference','Actual', 'Location', 'northeast');
    ylim([low_bd(i)-5, up_bd(i)+10]);
    xlim([0, time_plot{i}(end)+ 0.5]);
    ylabel('x position [mm]');
    xlabel('time [sec]');
    title("Ramp Signal Response (" + low_bd(i) + " mm to " + up_bd(i) + " mm, T = " + period(i) + " s)");
end

end

%% Function to plot the sinusoidal responses
function sinusoid_plotter_func(time_plot, x_des, x_act, freq, low_bd, up_bd)
% Description: this function is used to plot the sinusoid signals into one 
% figure as series of subplots (note that the maximum subplots you should
% plot for a single figure is 3 for clarity). 
% Inputs: the time vector (time_plot - cell), the desired and actual positions
% (x_des, x_act - cell), the frequency of the signal (freq - vector), the
% upper and lower bounds of the sine function (up_bd, low_bd -vector)
% Outputs: None

% Determine the number of plots by counting the number of cells
num_plots = length(time_plot);

% loop finds the lowest end time for the data so that the plot has uniform
% axes
lowest_time = time_plot{1}(end);
for j = 2:num_plots
    if time_plot{j}(end) < lowest_time
        lowest_time = time_plot{j}(end);
    end
end

figure

% loop to plot all the plots into a single figure
for i = 1:num_plots
    subplot(num_plots, 1, i);
    plot(time_plot{i},x_des{i},'LineWidth',1.5);        % reference position
    hold on 
    plot(time_plot{i},x_act{i},'LineWidth',1);        % actual position
    legend('Reference','Actual', 'Location', 'northeast');
    ylim([low_bd(i)-5, up_bd(i)+5]);
    xlim([0, lowest_time+0.5]);
    ylabel('x position [mm]');
    xlabel('time [sec]');
    title("Sinusoid Signal Response (" + low_bd(i) + " mm to " + up_bd(i) + " mm, f = " + freq(i) + " Hz)");
end

end