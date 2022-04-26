%% This code plots the fill time of the 1D channel as a function of solenoid PWM values
clear;
close all;
clc;

%% Read in the data from all the files
files = dir("*.csv");

% loop through the files and plot the respective data
for i=1:length(files)
    % obtain data
    filename = strcat(files(i).folder, '\', files(i).name);     % obtain file name
    data = readtable(filename);                                 % obtain data of a single file

    % transform times from ms to sec
    data.Var4 = data.Var4/1000;
    data.Var6 = data.Var6/1000;
    
    % plot each dataset
    figure
    plot(data.Var2, data.Var4, 'LineWidth', 1.5, 'Marker','.', 'MarkerSize',20);         % plots the inflate times
    hold on
    plot(data.Var2, data.Var6, 'LineWidth', 1.5, 'Marker', '.', 'MarkerSize',20);         % plots the deflate times

    experiment = files(i).name;                   % creates variable for title (type: char)
    exp_to_str = convertCharsToStrings(experiment);      % turns title into string type
    new_exp = erase(exp_to_str, ".csv");          % remove .csv extension for plotting
    title(new_exp);
    legend('Inflation', 'Deflation');
    xlabel('PWM Duty Cycle [%]');
    ylabel('Inflation/Deflation Time [s]');

    % save the figures as a png
    saveas(gcf, new_exp, 'png');
end
