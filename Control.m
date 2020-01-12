function [dx,u] = Control(s,r)
%CONTROL Takes a struct s representing a system and tracks reference r
%   Control takes a struct containing specific information about a system
%   and uses state feedback to try to get the states for the system to
%   converge to a reference value r.

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

