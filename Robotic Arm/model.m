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
    global Bsm Bsf Jm Jf Gc GR Ke Kf Kr Ks Kt L Lf Mf R g theta_U
    
    
    xdot(1,1) = (Va/L)-(R/L) * x(1) - (Ke/L) * x(3);
    xdot(2,1) = x(3);
    xdot(3,1) = (Kt/Jm) * x(1) + Bsm * x(3) - Bsm * x(5);
    xdot(4,1) = x(5);
    xdot(5,1) = (Bsm/Jm) * x(3) - (Bsm/Jm) * x(5);
    xdot(6,1) = x(7);
    xdot(7,1) = (GR*Kf*x(4)) - (Bsf/Jf) * x(7) - ((Mf*Lf)/(2*Jf))*g*sin(theta_U + x(6));
