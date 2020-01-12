function [s] = AngularDynamics
%ANGULARDYNAMICS Generates a struct s for use with the Control function
%   AngularDynamics generates a struct containing an abstract
%   representation of the missile dynamics. The dynamics use a point mass
%   model including forward velocity, angular position, and angular
%   acceleration. This file does NOT determine how the reference is
%   calculated; this should be handled by the calling function.

% A represents the plant dynamics
s.A = [0 0 0 ;
       0 0 1 ;
       0 0 0];
% B represents the controller dynamics
s.B = [1 0 ;
       0 0 ;
       0 1];

% Set state error wrapping 
% 1 indicates that state error should be wrapped to pi
% 0 indicates no wrapping
s.wrap = [0 1 0]; 
 
% Calculate the state feedback matrix
% p is the desired pole locations
% k is the resulting state feedback matrix
s.p = [-20 -25 -30];
s.k = place(s.A,s.B,s.p);

% States
% x(1) => Forward velocity
% x(2) => Angular position
% x(3) => Angular acceleration
s.x = [0 0 0]';

% Effort limits
s.limits = [-5 20;
            -5 5];

end

