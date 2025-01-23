% Bicopter Simulator
% Written by Matthew Santos
pkg load matgeom;
clear functions;
clear variables;
clc;
%close all;

% Global Varriables
t_frame = 0.1;  %[s] Simulation Resolution
d_frame = 500;  %[kg/m^3] Density (Wood,Pine)
t_end   = 4;    %[s] Simulation End Time
d       = 0.3;  %[m] Motor Seperation
h       = 0.05;  %[m] Motor Vertical Offset

% Define Bicopter Model
model = struct(
  'frame', struct(
    'density', d_frame,     %[kg/m^3] Density (Wood,Pine)
    'pos', [0;0;0],         %[m] Relative position
    'dim', [d;0.03;0.005],  %[m] Dimensions
    'color', "black"        %Display Color
  ),
  'L_motor', struct(
    'mass', 0.053,
    'pos', [-d/2;0;0.0397/2+h+0.005/2],
    'dim', [0.0279;0.0279;0.0397],
    'color', "blue"
  ),
  'R_motor', struct(
    'mass', 0.053,
    'pos', [d/2;0;0.0397/2+h+0.005/2],
    'dim', [0.0279;0.0279;0.0397],
    'color', "blue"
  ),
  'L_Standoff', struct(
    'density', d_frame,
    'pos', [d/2;0;h/2+0.005/2],
    'dim', [0.0279;0.0279;h],
    'color', "black"
  ),
  'R_Standoff', struct(
    'density', d_frame,
    'pos', [-d/2;0;h/2+0.005/2],
    'dim', [0.0279;0.0279;h],
    'color', "black"
  ),
  'Battery', struct(
    'mass', 0.179,
    'pos', [0;0;0.005/2+0.026/2],
    'dim', [0.108;0.036;0.026],
    'color', "red"
  )
);
motor = struct(
  'pos',[[d/2;0;0.0417+h],[-d/2;0;0.0417+h]], %[m] Location of Propellers
  'max_I', [9.6 9.6],           %[A] Max Current Draw
  'I_noLoad', [0.0165 0.0165],  %[A] No load Current
  'Kv', [935 935],              %[rpm/V] Motor KV Rating
  'rpm_poly', [20.97 -392.98 2547.5 0], %RPM vs Current PolyFit
  'thrust_poly', [-4.5802 109.93 0],           %Thrust vs Current PolyFit
  'prop_mass', [0.015 0.015],  %[kg]
  'prop_A', [0.0095219 0.0095219],  %[m^2]  (Lateral Cross Section)
  'prop_R', [0.13 0.13] %[m]  Propellar Radias
);

% Create Bicopter Model
bc = bicopter(model,motor);

% Set Initial Conditions
bc.x_ddot = [0 0 -bc.g];
bc.w = [0 0 0];
bc.w_dot = [0 0 0];

% Perform Analysis (for each frame)
% ----------------------
set(0, "defaultaxeslinewidth", 3);
set(0, "defaultaxesfontsize", 12);
set(0, "defaultlinelinewidth", 3);
set(0, "defaultlinemarkersize", 15);
function configplot(T,xl)
  grid on;
  title(T);
  xlabel(xl);
  legend('x','y','z');
endfunction
figure(1);clf;
view(3);hold on;grid on;
title("Bicopter Simulation");
xlim([-2 2]);xlabel("x [m]");
ylim([-2 2]);ylabel("y [m]");
zlim([-2 2]);zlabel("z [m]");
figure(2);clf;

% Begin Simulation
data=struct('t',[],'x',[],'x_dot',[],'x_ddot',[],'w',[],'w_dot',[],'w_ddot',[],'I',[],'rpm',[],'thrust',[]);
for t = 0:t_frame:t_end
  % Save Data
  data.t(end+1) = t;
  data.x(end+1,:) = bc.x;
  data.x_dot(end+1,:) = bc.x_dot;
  data.x_ddot(end+1,:) = bc.x_ddot;
  data.w(end+1,:) = bc.w;
  data.w_dot(end+1,:) = bc.w_dot;
  data.w_ddot(end+1,:) = bc.w_ddot;
  data.I(end+1,:) = bc.motor.I;
  data.rpm(end+1,:) = bc.motor.rpm;
  data.thrust(end+1,:) = bc.motor.thrust;
  % Plot Data
  figure(2);
  subplot(3,3,1);plot(data.t,data.x);configplot("Position [m]","Time");
  subplot(3,3,2);plot(data.t,data.x_dot);configplot("Velocity [m/s]","Time");
  subplot(3,3,3);plot(data.t,data.x_ddot);configplot("Acceleration [m/s^2]","Time");
  subplot(3,3,4);plot(data.t,data.w);configplot("Rotation [deg]","Time");
  subplot(3,3,5);plot(data.t,data.w_dot);configplot("Angular Velocity [deg/s]","Time");
  subplot(3,3,6);plot(data.t,data.w_ddot);configplot("Angular Acceleration [deg/s^2]","Time");
  subplot(3,3,7);plot(data.t,data.I);configplot("Motor Current [A]","Time");
  subplot(3,3,8);plot(data.t,data.rpm);configplot("Motor RPM","Time");
  subplot(3,3,9);plot(data.t,data.thrust);configplot("Motor Thrust [N]","Time");
  %pause(0.5);
  bc.plotState(1);
  bc = bc.updateState([0.25 0.25+0.00001*sind(180*t)],t_frame);
  if (bc.x(3) < -10); break;endif% Stop if we Crash
end


