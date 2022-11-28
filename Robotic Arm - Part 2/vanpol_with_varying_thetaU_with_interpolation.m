
% INITIAL SEGMENT

% clear the MATLAB workspace at the beginning of a program:
clear all

% Define and initialise the model input and any model parameters
% as global variables so that they can be read in the function model.m.

global Bsm Bsf Jm Jgear1 Jf Gc GR Ke Kf Kg Kr Ks Kt L Lf Mf R g theta_U

Gc = 0.5;           % elbow controller gain - value has been chosen for better control system using vanpol_with_varying_GC.m program
Bsm = 0.01;         % Nm/rad/s - damping coefficient for the motor
Bsf = 1.5;          % Nm/rad/s -  damping coefficient for the forearm
Jm = 0.002;         % kgm2 - moment of inertia for the motor armature
Jgear1= 0.001;      % kgm2 - moment of inertia for grear 1
Jf= 0.0204;         % kgm2 - total moment inertia of the forearm and hand
GR = 1.7;           % ratio between raduis of gear 1 and gear 2
Ke = 0.35;          % V/rad/s - back emf constant
Kf = 0.5;           % torque gain
Kg = 2.0;           % gear compensator gain
Kr= 0.9;            % reference amp gain
Ks= 0.9;            % motor deflection actuator sesor gain
Kt = 0.35;          % Nm/A - torque constant
L = 0.1;            % H - inductance
Lf = 0.35;          % m - length of forearm
Mf = 0.5;           % kg - masses of forearm
R = 4;              % Ω - resistance
g = 9.81;           % m s-2 - acceleration of gravity
Ki = 4;             % integration gain factor

% upper arm angle of deflection in radians:
% will vary with time in the find_response function:
theta_U = 0;

% reference angle in radians:
theta_Fref = deg2rad(55);

% Define parameters for the simulation
stepsize = 0.001;				% Integration step size
comminterval = 0.0025;			% Communications interval
EndTime = 25;					% Duration of the simulation (final time)
i = 0;							% Initialise counter for data storage

% x = initial states conditions:
% theta F equals theta gear 2 equals -9 degrees initially:
x = [0, 0, 0, 0, 0, deg2rad(-9), 0]';
% state derivagives:
xdot = [0, 0, 0, 0, 0, 0, 0]';

% find different values of Gc and plot them:
% clear plots:
for j=1:1:3
    figure(j)								% define figure window number
    clf										% clear figure
    title("MATLAB response")
    legend()
    xlabel('Time (s)')
    ylabel('Radians')
end

integrate_delta_theta = 0;

[tout, xout] = find_response(Gc, Ki, x, xdot, theta_Fref, Ks, Kg, stepsize, EndTime, comminterval, integrate_delta_theta);
% plot:
figure(1)
hold on
plot(tout,xout(:,2), 'DisplayName', "theta M") % theta M
hold off
figure(2)
hold on
plot(tout,xout(:,7), 'DisplayName', "theta F dot") % theta F dot
hold off
figure(3)
hold on
plot(tout,xout(:,6), 'DisplayName', "theta F") % theta F
hold off


figure(1)
yline(theta_Fref,"--", 'DisplayName','theta Fref target', 'color', [.3 .3 .3])

% END OF DYNAMIC SEGMENT

function [tout,xout] = find_response(Gc, Ki, x, xdot, theta_Fref, Ks, Kg, stepsize, EndTime, comminterval, integrate_delta_theta)
    
    % theta U will vary with time in the for loop:
    global theta_U

    % Va is the controller (defined here and at the end of the loop).
    % actuator voltage to drive the motor:
    Va = calc_Va(theta_Fref, Ks, x, integrate_delta_theta, stepsize, Gc, Ki, Kg);
    
    % END OF INITIAL SEGMENT - all parameters initialised
    i = 0;
    % DYNAMIC SEGMENT
    for time = 0:stepsize:EndTime
        % change the value of theta U on time intervals specified:
        theta_U = find_thetaU(time);

        % store time state and state derivative data every communication interval
        if rem(time,comminterval)==0
            i = i+1;					% increment counter
            tout(i) = time;	 	        % store time
            xout(i,:) = x;			    % store states - var name e.g xout0.001
            xdout(i,:) = xdot;	        % store state derivatives
        end							    % end of storage
    
        % DERIVATIVE SECTION
        xdot = model(x, Va);
        % END OF DERIVATIVE SECTION
    
        % INTEG SECTION
        % Numerical integration of the state derivatives for this time interval
        x = rk4int('model', stepsize, x, Va);
        % END OF INTEG SECTION
        
        Va = calc_Va(theta_Fref, Ks, x, integrate_delta_theta, stepsize, Gc, Ki, Kg);
        
    end
end

function Va = calc_Va(theta_Fref, Ks, x, integrate_delta_theta, stepsize, Gc, Ki, Kg)
    delta_theta = (theta_Fref * Ks) - (x(2) * Ks);
    integrate_delta_theta = eulerint(integrate_delta_theta, stepsize, delta_theta);
    Ve = (delta_theta * Gc) +  Gc * Ki * integrate_delta_theta;
    Va = Kg * Ve;
end

% Function to find the value of theta U at any given time using its Quadratic
% spline interpolation method:
% return the value of theta_U
function theta_U = find_thetaU(time)

    if 0 <= time && time <= 3.5
        theta_U = 22/49 * (time)^2 + time + 5;

    elseif 3.5 <= time && time <= 7.5
        theta_U = 5/56 * (time)^2 - 141/32 * time - 19/32;

    elseif 7.5 <= time && time <= 16.5
        theta_U = -74/189 * (time)^2 + 613/63 * time - 1111/42;

    elseif 16.5 <= time && time <= 21.5
        theta_U = 503/525 * (time - 16.5)^2 - 67/21 * (time - 16.5) + 27.5;

    elseif 21.5 <= time && time <= 25
        theta_U = -772/735 * (time - 21.5)^2 + 671/105 * (time - 21.5) + 35.5;
    
    end
end
% END OF SIMULATION PROGRAM
