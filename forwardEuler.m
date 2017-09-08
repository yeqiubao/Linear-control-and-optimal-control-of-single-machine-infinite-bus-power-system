function [t_, x_] = forwardEuler(f, t, x, h)
% f: state function
% t: timespan
% x: initial state
% u: control input 
% h: stepsize

% t_: time points
% x_: state trajectory
k = floor(t/h); % round down
x_ = zeros(2, k+1); % preallocate two state vectors
x_(:,1) = x; % initial condition
u_ = zeros(1, k+1); % zero input for this problem; you can change it to your control input
t_ = 0:h:k*h;
 for j = 1:k 
     x_(:,j+1) = x_(:,j)+h*f(x_(:,j), u_(:,j)); 
 end
end

