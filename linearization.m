clear all; close all; clc

global g l m
g = 9.81; l = 0.5; m = 1;

tspan = 2*pi*sqrt(l/g);
h = 1e-4; % stepsize
x0 = [1 0]';
n = length(x0);

%Use forward euler method to generate the state trajectory  
[t_, x_] = forwardEuler(@pendulum, tspan, x0, h); 
plot(t_, x_)
xlabel('time (sec)')
ylabel('angle, velocity')
%% a(i) 
% Get the derivative of the flow with respect to state by solving an LTV 
% equation, i.e., dX(s)=A(s)X(s), X(0) = I

X_ = eye(n); % set X(0)=I
for j = 1: floor(tspan/h)
    X_ = X_+h*linearized_pendulum(X_, x_(:,j)); % Use forward Euler to solve
end

X_

%% a(ii) 
% Get the derivative of the flow with respect to state by computing DxPsi
% through finite-central-differences

dt = 1e-6;
DxPsi = zeros(n, n); % preallocate DxPsi
I = eye(n);
for i = 1:n
    DxPsi(:,i) = 0.5/dt*(psi_flow(@pendulum, tspan, x0+dt*I(:,i), h) - ...
                        psi_flow(@pendulum, tspan, x0-dt*I(:,i), h));
end

DxPsi

%% Compare the results of (i) and (ii); 
% Here I use matrix 1-norm, you can choose to use 2-norm or Inf-norm
norm_diff = norm(X_-DxPsi, 1)


