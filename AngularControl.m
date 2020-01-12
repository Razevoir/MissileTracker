function [dx,u] = AngularControl(A,B,x,k,r,limits)
%ANGULARDYNAMICS Summary of this function goes here
%   Detailed explanation goes here

% Determine the error
xh = x-r;
xh(2) = wrapToPi(xh(2));

% Calculate control input
u = -k*xh;

% Apply limits
for i=1:size(limits,1)
    u(i) = median([limits(i,1) u(i) limits(i,2)]);
end

% define results
dx = A*x+B*u; % change in states

end

