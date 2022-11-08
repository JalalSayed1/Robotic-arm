% This function represents the dynamic equations for Van der Pol's oscillator 
% 
% This is the DERIVATIVE SECTION of the simulation.
%
% The current time, state and input values are passed to the function as arguments
% and the function returns the state derivative.
% 
% x = states, Va actuator voltage to drive the motor: 
function xdot = model(x, Va)

    % global parameter transferred from main program
    global Bsm Bsf Jm Jgear1 Jf Gc GR Ke Kf Kr Ks Kt L Lf Mf R g theta_U
    
    
    xdot(1,1) = 1/L * (Va-R * x(1) - Ke * x(3));
    xdot(2,1) = x(3);
    xdot(3,1) = 1/Jm*(Kt*x(1)-Bsm*(x(3)-x(5)));
    xdot(4,1) = x(5);
    xdot(5,1) = 1/Jgear1*(Bsm*(x(3)-x(5)));
    xdot(6,1) = x(7);
    %xdot(7,1) = (GR*Kf*x(4)) - (Bsf/Jf) * x(7) - ((Mf*Lf)/(2*Jf))*g*sin(theta_U + x(6));
    xdot(7,1) = 1/Jf *((GR * Kf * x(4)) - (Bsf * x(7)) - ((Mf * Lf * g * sin(theta_U + x(6))) / 2));
