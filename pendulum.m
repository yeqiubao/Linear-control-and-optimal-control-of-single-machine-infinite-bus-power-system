function dx = pendulum(x,u)
global g l m
dx = zeros(2,1); % allocate a column vector
dx(1) = x(2);
dx(2) = -g/l*sin(x(1))+1/m*u;