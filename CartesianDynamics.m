function [s] = CartesianDynamics
%ANGULARDYNAMICS Generates a struct s for use with the Control function
%   CartesianDynamics generates a struct containing an abstract
%   representation of the missile dynamics. The dynamics use a point mass
%   model including horizontal position, horizontal acceleration, vertical
%   position, and vertical acceleration. This file does NOT determine how
%   the reference calculated; this should be handled by the calling
%   function.

% A represents the plant dynamics
s.A = [0 1 0 0 ;
       0 0 0 0 ;
       0 0 0 1 ;
       0 0 0 0];
% B represents the controller dynamics
s.B = [0 0 ;
       1 0 ;
       0 0 ;
       0 1];

% Set state error wrapping
% 1 indicates that state error should be wrapped to pi
% 0 indicates no wrapping
s.wrap = [0 0 0 0]; % Indicates that the second state should be wrapped to pi

% Calculate the state feedback matrix
% p is the desired pole locations
% k is the resulting state feedback matrix
s.p = [-20 -25 -30 -35];
s.k = place(s.A,s.B,s.p);

% States
% x(1) => x position
% x(2) => x velocity
% x(3) => y position
% x(4) => y velocity
s.x = [0 0 0 0]';

% Effort limits
s.limits = [-20 20;
            -20 20];

end

