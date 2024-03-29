
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
Gc = 9.95;          % elbow controller gain
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
comminterval = 0.001;			% Communications interval
EndTime = 20;					% Duration of the simulation (final time)
i = 0;							% Initialise counter for data storage

% x = initial states conditions:
% theta F equals theta gear 2 equals -9 degrees initially:
x = [0, 0, 0, 0, 0, deg2rad(-9), 0]';
% state derivagives:
xdot = [0, 0, 0, 0, 0, 0, 0]';
    
% Va is the controller (defined here and at the end of the loop).
% actuator voltage to drive the motor:
Va = Kg * Gc * ((theta_Fref * Ks) - (x(2) * Ks));

% END OF INITIAL SEGMENT - all parameters initialised

% DYNAMIC SEGMENT
for time = 0:stepsize:EndTime
    % store time state and state derivative data every communication interval
    if rem(time,comminterval)==0
        i = i+1;					% increment counter 
        tout(i) = time;	 	        % store time
        xout(i,:) = x;			    % store states
        xdout(i,:) = xdot;	        % store state derivatives
    end							    % end of storage      
   
    % DERIVATIVE SECTION
	
    xdot = model(x, Va);

	% END OF DERIVATIVE SECTION
	
	% INTEG SECTION
	
    % Numerical integration of the state derivatives for this time interval
    x = rk4int('model', stepsize, x, Va);
    % END OF INTEG SECTION

    Va = Kg * Gc * ((theta_Fref * Ks) - (x(2) * Ks));
    
end


% END OF DYNAMIC SEGMENT

% TERMINAL SEGMENT
figure(1)										% define figure window number
clf												% clear figure
hold on
% grid on

plot(tout,xout(:,2)) % theta M
plot(tout,xout(:,6)) % theta F
plot(tout,xout(:,7)) % theta F dot

title("MATLAB response")
%       theta M         theta F         theta F dot
legend('Motor angle', 'Forearm angle', 'Speed of forearm')
xlabel('Time (s)')
ylabel('Radians')

hold off

% END OF TERMINAL SECTION

% END OF SIMULATION PROGRAM
    