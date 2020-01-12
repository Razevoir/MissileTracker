function [dx,u] = Control(s,r)
%ANGULARDYNAMICS Summary of this function goes here
%   Detailed explanation goes here

% Determine the error
xh = s.x-r;

for i=1:size(s.wrap,2)
    if s.wrap(i)
        xh(i) = wrapToPi(xh(2));
    end
end

% Calculate control input
u = -s.k*xh;

% Apply limits
for i=1:size(s.limits,1)
    u(i) = median([s.limits(i,1) u(i) s.limits(i,2)]);
end

% define results
dx = s.A*s.x+s.B*u; % change in states

end

