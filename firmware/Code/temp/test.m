% Written by Matthew Santos
clear;
clc;
%close all;

% Global Varriables
t_frame = 0.1;  %[s] Simulation Resolution
d_frame = 500;  %[kg/m^3] Density (Wood,Pine)
t_end   = 4;    %[s] Simulation End Time
d       = 0.3;  %[m] Motor Seperation
h       = 0.05;  %[m] Motor Vertical Offset

x_raw = linspace(-1000,1000,100);
y_raw = linspace(-1000,1000,100);
z_raw = linspace(0,1000,100);
r_raw = linspace(-1000,1000,100);


[z,y] = meshgrid(z_raw,y_raw);

MF = 10000;

%Percent Throtles

L = ((512/125)*z - (512/625)*y);
R = ((512/125)*z + (512/625)*y);


L = min(max(L,0),4096);
R = min(max(R,0),4096);



set(0, "defaultaxeslinewidth", 3);
set(0, "defaultaxesfontsize", 12);
set(0, "defaultlinelinewidth", 3);
set(0, "defaultlinemarkersize", 15);
figure(1);clf;
colormap("default");
surf(z,y,L);grid on;hold on;surf(z,y,R);
xlabel("z");ylabel("y");zlabel("Thrust");
xlim([0 1000]);
ylim([-1000 1000]);
zlim([0 Inf]);
%zlim([-1 1]);
##subplot(1,2,1);plot(z,L);xlabel("z");ylabel("Thrust");grid on;hold on;
##plot(z,R);plot(z,L_L);plot(z,R_L);
##subplot(1,2,2);plot(y,L);xlabel("y");ylabel("Thrust");grid on;hold on;
##plot(y,R);plot(y,L_L);plot(y,R_L);

