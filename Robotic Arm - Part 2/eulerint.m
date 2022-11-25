function x = eulerint(xdot, h, x)
    % This function performs one Euler integration step where xdot represents
    % the state derivatives from a state space model and h is the step size
    x = x + h*xdot;