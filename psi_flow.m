function x_t = psi_flow(f, t, x, h)
% This function shows the flow Psi(t,x) is just x(t)
% f: state function
% t: timespan
% x: initial state
% h: stepsize
[~, x_] = forwardEuler(f, t, x, h); 
x_t = x_(:,end);
end
