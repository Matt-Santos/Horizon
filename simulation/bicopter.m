# Bicopter Class
# Written by Matthew Santos


classdef bicopter

  % Model Input Parameters
  properties (Access = public)
    elements = {};  % Stores Bicopter Components
  endproperties

  % Static Parameters
  properties (Access = private)
    CoM = zeros(3,1);
    Total_Mass = 0;
    I_Matrix = zeros(3);
  endproperties

  % State Varriables
  properties (Access = private)
    x =       zeros(3,1); % Position Vector
    x_dot =   zeros(3,1); % Velocity Vector
    x_ddot =  zeros(3,1); % Acceleration Vector
    w =       zeros(3,1); % Rotation Vector
    w_dot =   zeros(3,1); % Angular Velocity Vector
    w_ddot =  zeros(3,1); % Angular Acceleration Vector
  endproperties

  methods

    % Bicopter Constructor
    function bc = bicopter(model)
      bc.elements = model; % Load Model Data
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
      % Calculate Inertia Matrix
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
    function plotState(i)
      figure(i); hold on;
      %plot bicopter object
      for [element, key] = bc.elements

        drawBox3d([XMIN XMAX YMIN YMAX ZMIN ZMAX]);
      end

      %plot velocity
      %plot acceleration

      %update position Trail
      %update vectors...
    end

    % Calculate the Thrust Coeficient for a Given Motor
  endmethods
endclassdef
