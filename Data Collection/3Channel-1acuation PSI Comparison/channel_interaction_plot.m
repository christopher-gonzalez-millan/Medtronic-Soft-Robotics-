%% Quick Script for 3 Channel Pressure Interaction

clear
close all
clc

%% Read in the data from all the files
files = dir('NonStateMachine/*.csv');

% loop through the files and plot the respective data
for i=1:length(files)
    % obtain data
    filename = strcat(files(i).folder, '\', files(i).name);     % obtain file name
    data = readtable(filename);                                 % obtain data of a single file
    
    % plot each dataset
    figure
    plot(data.elpasedTime, data.actualPSI_Channel0, 'LineWidth', 1.5, 'Marker','.', 'MarkerSize',20);         % channel 0 pressure
    hold on
    plot(data.elpasedTime, data.actualPSI_Channel1, 'LineWidth', 1.5, 'Marker', '.', 'MarkerSize',20);         % channel 1 pressure
    plot(data.elpasedTime, data.actualPSI_Channel2, 'LineWidth', 1.5, 'Marker', '.', 'MarkerSize', 20);        % channel 2 pressure

    title("Pressure Change in Channels (Varying Channel: " + (i-1) + ")");
    legend('Channel 0', 'Channel 1', 'Channel 2', 'Location', 'northwest');
    xlabel('Elapsed Time [s]');
    ylabel('Pressures [psi]');

    % save the figures as a png
    saveas(gcf, "channel_" + (i-1), 'png');
end
