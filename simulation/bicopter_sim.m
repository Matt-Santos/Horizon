% Bicopter Simulator
% Written by Matthew Santos
clear;
clc;
close all;

% Global Varriables
t_frame = 0.25; %[s] Simulation Resolution
t_end   = 1000; %[s] Simulation End Time
d       = 0.3;  %[m] Motor Seperation
k       = 0.02; %[m] Battery Vertical Displacement

% Define Bicopter Model (SI Units)
model = struct(
  'frame', struct(
    'density', 500, %Wood,Pine
    'pos', [0;0;0],
    'dim', [d;0.03;0.005]
  ),
  'L_motor', struct(
    'mass', 0.053,
    'pos', [-d/2;0;0],
    'dim', [0.0279;0.0279;0.0397]
  ),
  'R_motor', struct(
    'mass', 0.053,
    'pos', [d/2;0;0],
    'dim', [0.0279;0.0279;0.0397]
  ),
  'Battery', struct(
    'mass', 0.1,
    'pos', [0;0;k],
    'dim', [0.03;0.03;0.03]
  )
);

##% Define Motor Characteristics
##bicopter.motor = {
##  mass = 53 %[g]
##  peakThrust = 850 %[G]
##  kv_Rating = 935 %[KV]
##  Prop = {
##    type = "EMAX-1045"
##    diameter = 0.254 %[m]
##    blade_Count = 2
##    pitch = 0.1143 %[m]
##    rotation = "CW"
##  }
##};

% Create Bicopter Model
bc = bicopter(model);

% Perform Analysis (for each frame)
% ----------------------
bc.plotState(1);



% Start Simulating
##for t= 0:t_frame:t_end
##  % Define Control Input here...
##  Control_State = [0,0]; % Two Motors at Percent of Full Throttle
##
##  % Calculate Accelerations
##
##
##  % Update Position State
##  x_dot = x_dot+x_ddot*t_frame;
##  x = x+x_dot*t_frame;
##  % Update Rotation State
##  w_dot = w_dot+w_ddot*t_frame;
##  w = w+w_dot*t_frame;
##  % Update Plots
##    %bicopterPlot(x,x_dot,x_ddot,w,w_dot,w_ddot,t);
##  pause(t_frame);
##end

