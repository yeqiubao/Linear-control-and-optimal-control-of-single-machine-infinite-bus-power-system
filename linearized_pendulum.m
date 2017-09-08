function dX = linearized_pendulum(X, x_j)
% X: derivative of the flow with respect to state
% x_j: state at t_j
global g l 

% syms u; 
% f = @(x)[x(2), -g/l*sin(x(1))+1/m*u];
% [As, ~] = jacobianest(f, x_j);

As = [0, 1; -g/l*cos(x_j(1)) 0];
dX = As*X;
end