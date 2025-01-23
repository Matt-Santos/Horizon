# Bicopter Class
# Written by Matthew Santos

classdef bicopter

  % Model Input Parameters
  properties
    elements =  {}; % Stores Bicopter Components
    motor =    {
      setpoint = zeros(1,3),  % Normalized Motor Control Setpoints [-1,1]
      I = zeros(1,2),
      rpm = zeros(1,2),
      thrust = zeros(1,2),
      accel = zeros(1,3),
      max_I = zeros(1,3),
      I_noLoad = zeros(1,3),
      prop_mass = zeros(1,3),
      prop_A = zeros(1,3),
      prop_R = zeros(1,3)
    }; % Stores Motor Specific Parameters
  endproperties

  % Static Parameters
  properties
    CoM = zeros(3,1);
    Total_Mass = 0;
    I_Matrix = zeros(3);  % Inertia Matrix about center of mass
  endproperties

  % State Varriables
  properties
    dt =      0;          % Time Step
    x =       zeros(1,3); % Position Vector
    x_dot =   zeros(1,3); % Velocity Vector
    x_ddot =  zeros(1,3); % Acceleration Vector
    torque =  zeros(1,3); % Rotational Torques
    w =       zeros(1,3); % Rotation Vector
    w_dot =   zeros(1,3); % Angular Velocity Vector
    w_ddot =  zeros(1,3); % Angular Acceleration Vector
    Mw_dot =  zeros(1,2); % Motor Angular Veclocity Vector
    Mw_ddot = zeros(1,2); % Motor Angular Acceleration Vector
  endproperties

  % Control Varriables
  properties

  endproperties

  % System Constants
  properties
    g = 9.80665; %[m/s^2] acceleration of gravity
    k_air = 1.05; %[] air resistance factor for cube
  endproperties

  methods
    % Bicopter Constructor
    function bc = bicopter(model,motor)
      bc.elements = model;  % Load Model Data
      bc.motor = motor;     % Load Motor Specs
      % Initialize Motor Properties
      bc.motor.setpoint = zeros(1,3);
      bc.motor.I = zeros(1,2);
      bc.motor.rpm = zeros(1,2);
      bc.motor.thrust = zeros(1,2);
      bc.motor.accel = zeros(1,3);
      % Calculate Center of Mass
      for [element, key] = bc.elements
        if not(isfield(element,'mass')) && isfield(element,'density')
          mass = element.density*prod(element.dim);
          density = element.density;
          bc.elements = setfield(bc.elements,key,'mass',mass);
        elseif isfield(element,'mass') && not(isfield(element,'density'))
          mass = element.mass;
          density = mass/prod(element.dim);
          bc.elements = setfield(bc.elements,key,'density',density);
        else
          display("Error Missing Mass/Density Model Data");
        endif
        bc.CoM += mass*element.pos;
        bc.Total_Mass += mass;
      end
      bc.CoM = bc.CoM/bc.Total_Mass;
      % Calculate Inertia Matrix (improve, should integrate of area instead)
      for [element, key] = bc.elements
        r_CoM = bc.CoM-element.pos;
        bc.I_Matrix += element.mass*[
          r_CoM(2)^2+r_CoM(3)^2 -r_CoM(1)*r_CoM(2) -r_CoM(1)*r_CoM(3);
          -r_CoM(1)*r_CoM(2) r_CoM(1)^2+r_CoM(3)^2 -r_CoM(2)*r_CoM(3);
          -r_CoM(1)*r_CoM(3) -r_CoM(2)*r_CoM(3) r_CoM(1)^2+r_CoM(2)^2
        ];
      end
    endfunction

    % Plot the current state of the Bicopter to figure i
    function plotState(bc,i)
      figure(i); hold on;
      % Draw Bicopter
      rot_matrix = eulerAnglesToRotation3d(bc.w(1),bc.w(2),bc.w(3));
      for [element, key] = bc.elements
        pos_rot = transformPoint3d(element.pos',rot_matrix);
        xc = bc.x(1)+pos_rot(1);
        yc = bc.x(2)+pos_rot(2);
        zc = bc.x(3)+pos_rot(3);
        b = drawCuboid([xc yc zc element.dim(1) element.dim(2) element.dim(3) bc.w(1) bc.w(2) bc.w(3)],'FaceColor',element.color);
      end
      % Draw Propellers
      for ii = 0:length(bc.motor.prop_R)-1
        pos = [bc.motor.pos(1+3*ii) bc.motor.pos(2+3*ii) bc.motor.pos(3+3*ii)];
        pos_rot = transformPoint3d(pos,rot_matrix);
        xc = bc.x(1)+pos_rot(1);
        yc = bc.x(2)+pos_rot(2);
        zc = bc.x(3)+pos_rot(3);
        v = transformPoint3d([0 0 0.001],rot_matrix);
        drawCylinder([xc yc zc xc+v(1) yc+v(2) zc+v(3) bc.motor.prop_R(ii+1)*bc.motor.setpoint(ii+1)]);
      end
      % Plot kinematic vectors
      n = 0.5;  %[m] Normalized vector length (to improve display)
      forward = transformPoint3d([0 1 0],rot_matrix); % Forward Facing Vector
      drawVector3d(bc.x, n*forward/norm(forward),'Color',"black");
      drawVector3d(bc.x, n*bc.x_dot/norm(bc.x_dot),'Color',"green");
      drawVector3d(bc.x, n*bc.x_ddot/norm(bc.x_ddot),'Color',"red");
    endfunction

    % Update kinematics
    function bc = updateState(bc,ctrls,dt)
      bc.dt = dt;
      % Update Motor with new Throttle Setting
      bc = updateMotor(bc,ctrls);
      % Rotational Dynamics
      bc.w_ddot = (inv(bc.I_Matrix)*(bc.torque') - inv(bc.I_Matrix)*cross(bc.w_dot',(bc.I_Matrix*bc.w_dot')))';
      bc.w_dot = bc.w_dot + bc.w_ddot*dt;
      bc.w = bc.w + bc.w_dot*dt;
      % Linear Dynamics
      bc.x_ddot = bc.calc_x_ddot();
      bc.x_dot = bc.x_dot + bc.x_ddot*dt;
      bc.x = bc.x + bc.x_dot*dt;
    endfunction

    % Update the Motor State
    function bc = updateMotor(bc,ctrls)
      bc.motor.setpoint = ctrls;
      rot_matrix = eulerAnglesToRotation3d(bc.w(1),bc.w(2),bc.w(3));
      % Calculate Motor Parameters from Setpoint
      bc.motor.I = bc.motor.setpoint.*bc.motor.max_I;                 %[A]
      bc.motor.rpm = polyval(bc.motor.rpm_poly,bc.motor.I);           %[rpm]
      bc.motor.thrust = polyval(bc.motor.thrust_poly,bc.motor.I);     %[g]
      % Convert to Standard Measurements
      Mw_dot_old = bc.Mw_dot;
      bc.Mw_dot = (2*pi/60)*(180/pi)*bc.motor.rpm;               %[deg/s]
      bc.motor.thrust = (bc.g/1000)*bc.motor.thrust;                  %[N]
      % Calculate Motor Angular Acceleration
      bc.Mw_ddot = (bc.Mw_dot - Mw_dot_old)/bc.dt;
      % Calculate Thrust based Acceleration

          %this needs to be modified to the sumation about the center of mass
      bc.motor.accel = transformPoint3d([0 0 sum(bc.motor.thrust)/bc.Total_Mass], rot_matrix);

      % Calculate Air friction on propellers
      C_d = 0.05;  %Rough Estimate
      A = bc.motor.prop_A;
      R = bc.motor.prop_R;
      rho = calc_air_density(bc,bc.x(3));
      Fp_friction = 0.5*rho*C_d*A.*(bc.Mw_dot.*R).^2;
      % Calculate Rotational Torque of props on Motors
      n = 2;  % Number of blades per prop
      m = bc.motor.prop_mass;
      L = bc.motor.prop_R;
      Inertia = n*(m.*L.^2)/3; %Inertia along motor axis (assumes cylinder shaped)
      R_Torque = Fp_friction + Inertia.*bc.Mw_ddot;
      R_Torque = R_Torque.*[1 1];  %Reverse the Second Propeller??? (why is this wrong?)
      % Calculate Torques on Bicopter Caused by Motors
      torque = zeros(1,3);
      for i=1:length(bc.motor.thrust)
        r = bc.motor.pos(1+(3*(i-1)):3+(3*(i-1)))-bc.CoM';
        Ft = [0 0 bc.motor.thrust(i)];
        Fr = [0 0 R_Torque(i)];
        torque += cross(r,Ft); % Differential Thrust Torque
        torque += cross(r,Fr); % Change in Rotational Inertia Torque
      end
      bc.torque = transformPoint3d(torque,rot_matrix); % Rotate to Ref Frame
    endfunction

    % Calculate Acceleration Vector
    function x_ddot = calc_x_ddot(bc)
      % Define Force Vectors
      gravity = [0 0 -bc.g];
      air_resistance = bc.calc_air_resistance();
      motor_accel = bc.motor.accel;
      % Perform Calculation
      x_ddot = (1/bc.Total_Mass)*motor_accel+gravity+air_resistance;
    endfunction

    % Calculate Air Resistance
    function a_air = calc_air_resistance(bc)
      % Calculate Cross Section along each axis
      A = zeros(1,3);
      for [element, key] = bc.elements
        A(1) += element.dim(2)*element.dim(3);
        A(2) += element.dim(1)*element.dim(3);
        A(3) += element.dim(1)*element.dim(2);
      end
      rot_matrix = eulerAnglesToRotation3d(bc.w(1),bc.w(2),bc.w(3));
      A = transformPoint3d(A, rot_matrix);
      % Perform the Calculation
      rho_air = bc.calc_air_density(bc.x(3));
      a_air = -(0.5*rho_air*bc.k_air)*A.*bc.x_dot;
    endfunction

    % Calculate Air Density
    function rho = calc_air_density(bc,ASL)
      % Determine Air pressure and temperature at ASL
      p = 101325*(1-0.0065*ASL/288.15)^(bc.g*0.0289652/(8.31446*0.0065)); %[Pa]
      T = 288.15 - 0.0065*ASL;  %[K]
      % Determine Air density
      rho = p*0.0289652/(8.31446*T);  %[kg/m^3]
    endfunction

  endmethods
endclassdef
