function [s] = AngularDynamics
%ANGULARDYNAMICS Summary of this function goes here
%   Detailed explanation goes here
s.A = [0 0 0 ;
       0 0 1 ;
       0 0 0];
s.B = [1 0 ;
       0 0 ;
       0 1];
 
s.p = [-20 -25 -30]; % desired pole locations
s.k = place(s.A,s.B,s.p); % state feedback gains

% Initial conditions
s.vi = 5;
s.wi = pi/2;

% States
% x(1) => Forward velocity
% x(2) => Angular position
% x(3) => Angular acceleration
s.x = [s.vi, s.wi, 0]';

% Effort limits
s.limits = [-1 3;-1.5 1.5];

end

