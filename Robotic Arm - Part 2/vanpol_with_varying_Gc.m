
% INITIAL SEGMENT

% clear the MATLAB workspace at the beginning of a program:
clear all

% Define and initialise the model input and any model parameters
% as global variables so that they can be read in the function model.m.

global Bsm Bsf Jm Jgear1 Jf Gc GR Ke Kf Kg Kr Ks Kt L Lf Mf R g theta_U

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

% initial conditions:
% upper arm angle of deflection in radians,
% assumed fixed for this assignemen:
theta_U = deg2rad(5);

% reference angle in radians:
theta_Fref = deg2rad(55);

% Define parameters for the simulation
stepsize = 0.001;				% Integration step size
comminterval = 0.0025;			% Communications interval
EndTime = 20;					% Duration of the simulation (final time)
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

figure(1)
yline(theta_Fref,"--", 'DisplayName','theta Fref target')

% Gc = elbow controller gain - this value will change in a for loop to investigate best value for it
for Gc=0.4:0.05:0.6
    [tout, xout_Gc] = find_response(Gc, x, xdot, theta_Fref, Ks, Kg, stepsize, EndTime, comminterval);
    % plot:
    Gc_str = num2str(Gc);
    figure(1)
    hold on
    plot(tout,xout_Gc(:,2), 'DisplayName', " Gc = " + Gc_str) % theta M
    hold off
    figure(2)
    hold on
    plot(tout,xout_Gc(:,7), 'DisplayName', "theta F dot "+ Gc_str) % theta F dot
    hold off
    figure(3)
    hold on
    plot(tout,xout_Gc(:,6), 'DisplayName', "theta F "+ Gc_str) % theta F
    hold off
end

% END OF DYNAMIC SEGMENT

function [tout,xout] = find_response(Gc, x, xdot, theta_Fref, Ks, Kg, stepsize, EndTime, comminterval)
    % Va is the controller (defined here and at the end of the loop).
    % actuator voltage to drive the motor:
    Ve = Gc * ((theta_Fref * Ks) - (x(2) * Ks));
    Va = Kg * Ve;
    
    % END OF INITIAL SEGMENT - all parameters initialised
    i = 0;
    % DYNAMIC SEGMENT
    for time = 0:stepsize:EndTime
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
    
        Ve = Gc * ((theta_Fref * Ks) - (x(2) * Ks));
        Va = Kg * Ve;
    
    end
end
% END OF SIMULATION PROGRAM
