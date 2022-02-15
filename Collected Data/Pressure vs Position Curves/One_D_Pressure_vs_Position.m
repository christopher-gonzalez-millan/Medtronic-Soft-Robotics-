%% Pressure vs Position Plot for the 1D Channel

clear
clc
close all

%% Import the data from spreadsheet
[P_input, P_sensor, x_EM, y_EM] = readvars('1D Pressure vs Pos.xlsx');
%% Flip the x-values for plotting
x_EM = -1*x_EM;

%% Plot the data as position vs pressure in the x-direction and y-directions
figure

% position vs pressure in x-direction
subplot(2, 1, 1);
plot(P_input,x_EM,'MarkerFaceColor',[0 0.447058823529412 0.741176470588235],...
    'MarkerSize',4,...
    'Marker','o',...
    'LineWidth',1);
hold on 
plot(P_sensor,x_EM,'MarkerFaceColor',[0.850980392156863 0.325490196078431 0.0980392156862745],...
    'MarkerSize',4,...
    'Marker','o',...
    'LineWidth',1);
legend('Input pressure','Sensor Pressure');
ylabel('x position [mm]');
xlabel('Pressure [psi]');
title({'Pressure vs Position of 1D Channel'});

% position vs pressure in y-direction
subplot(2, 1, 2);
plot(P_input,y_EM,'MarkerFaceColor',[0 0.447058823529412 0.741176470588235],...
    'MarkerSize',4,...
    'Marker','o',...
    'LineWidth',1);
hold on
plot(P_sensor,y_EM,'MarkerFaceColor',[1 0 0],...
    'MarkerSize',4,...
    'Marker','o',...
    'LineWidth',1);
legend('Input pressure','Sensor Pressure','Position', ...
    [0.169880800985155 0.339053362945055 0.235357146297182 0.0754761920202346]);
ylabel('y position [mm]');
xlabel('Pressure [psi]');

%% Plot the x vs y positions
figure
plot(x_EM, y_EM, 'MarkerFaceColor',[0 0.447058823529412 0.741176470588235],...
    'MarkerSize',4,...
    'Marker','o',...
    'LineWidth',1);
xlabel('x position [mm]');
ylabel('y position [mm]');
title('End effector positions')