%% Constrained least squares test
% This script tests the least squares algorithm for the 3 channel feedback
% control algorithm
% NOTE: Testing in this script reveals that introducing bounds causes the
% solutions to be reduced by a factor of 3.666 for even relatively large
% bounds

clear; clc; close all;

xlrange = -90; %x lower range
xurange = 90; %x upper range
ylrange = -90; %y lower range
yurange = 90; %y upper range

bu = 15; %max input 
bl = -15; %min input

bxsign = randi([1,2]);
bysign = randi([1,2]);
bx = ((-1) ^ (bxsign)) * (xlrange + (xurange - xlrange)) * rand(); %Randomize x desired
by = ((-1) ^ (bysign)) * (ylrange + (yurange - ylrange)) * rand(); %Randomize y desired

Ax = [sqrt(3)/2 -sqrt(3)/2 0]; %A values in x
Ay = [1/2 1/2 -1]; %A values in y
I = eye(3); %Identity matrix size of Ax and Ay

bmax = bu * ones(3,1); %max inputs
bmin = bl * ones(3,1); %min inputs

A = cat(1, Ax, I, I, Ay, I, I); %Create A for Bounded case
b = cat(1, bx, bmax, bmin, by, bmax, bmin); %Create b for Bounded case

tic
x = pinv(A)*b; %Compute m,n,o for bounded case
toc

A2 = cat(1, Ax, Ay); %Create A for unbounded
b2 = cat(1, bx, by); %Create b for unbounded

tic
x2 = pinv(A2)*b2; %Solve m,n,o for unbounded
toc

%% Verify the calculated solutions are accurate
e = cat(1, Ax, Ay)*x;       % bounded
e2 = cat(1, Ax, Ay)*x2;     % unbounded

sf_x = x2./x;               % scale factor between the solutions (m, n, o)
sf_e = e2./e;               % scale factor for the resolved error vector (using the calc m, n, o)