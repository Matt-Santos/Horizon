% Bicopter Simulator
% Written by Matthew Santos
clear;clc;

% Global Varriables
t_frame = 0.25; %[s] Simulation Resolution
t_end   = 1000; %[s] Simulation End Time

% Create Bicopter Model
%-----------------------
% Element Dimensions [m] (x,y,z)
bicopter.frame.dim   = [0.3,0.03,0.005];
bicopter.L_motor.dim = [0.0279,0.0279,0.0397];
bicopter.R_motor.dim = bicopter.L_motor.dim;
bicopter.Battery.dim = [0.3,0.3,0.3];
% Element Relative Positions [m] (x,y,z)
bicopter.frame.pos   = [0,0,0];
bicopter.L_motor.pos = [-bicopter.frame.dim(1)/2,0,0];
bicopter.R_motor.pos = [ bicopter.frame.dim(1)/2,0,0];
bicopter.Battery.pos = [0,0,0];
% Element Densities [kg/m^3]
bicopter.frame.density   = 500; %Wood,Pine
bicopter.L_motor.density = (53/1000)/prod(bicopter.L_motor.dim);  %Calculated
bicopter.R_motor.density = bicopter.L_motor.density;
bicopter.Battery.density = (100/1000)/prod(bicopter.Battery.dim); %Calculated
% Calculate Total Mass
for element=bicopter(:)
  Total_Mass += element.density*prod(element);
  Center_of_Mass = [];
end

% Calculate Inertia Matrix
I = []; %(3x3 matrix)



% Define Motor Characteristics
bicopter.motor = {
  mass = 53 %[g]
  peakThrust = 850 %[G]
  kv_Rating = 935 %[KV]
  Prop = {
    type = "EMAX-1045"
    diameter = 0.254 %[m]
    blade_Count = 2
    pitch = 0.1143 %[m]
    rotation = "CW"
  }
};

% Perform Analysis (for each frame)
% ----------------------
% Define Initial System State
x =       zeros(3,1); % Position Vector
x_dot =   zeros(3,1); % Velocity Vector
X_ddot =  zeros(3,1); % Acceleration Vector
w =       zeros(3,1); % Rotation Vector
w_dot =   zeros(3,1); % Angular Velocity Vector
w_ddot =  zeros(3,1); % Angular Acceleration Vector
% Start Simulating
for t= 0:t_frame:t_end
  % Define Control Input here...
  Control_State = [0,0]; % Two Motors at Percent of Full Throttle

  % Calculate Accelerations


  % Update Position State
  x_dot = x_dot+x_ddot*t_frame;
  x = x+x_dot*t_frame;
  % Update Rotation State
  w_dot = w_dot+w_ddot*t_frame;
  w = w+w_dot*t_frame;
  % Update Plots
  bicopterPlot(x,x_dot,x_ddot,w,w_dot,w_ddot,t);
  delay();
end

% Program Functions
% ----------------------
% Calculate the Thrust Coeficient for a Given Motor
function Thrust()

end

% Cal

function bicopterPlot()
  %plot current position
  %update position Trail
  %update vectors...
end

