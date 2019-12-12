clc; clear all; close all;

%% define target
% can define target as either desired velocity and angle for the missile...
% ... or as a position in 2d space to try and hit during flight

% target location in cartesian coordinates
global xt
global yt

xt = 10;
yt = 40;

% time to hit the target
global t_hit
t_hit = 10;

% missile location in cartesian coordinates
global xm
global ym

xm = 0;
ym = 0;

global vd % desired forward velocity
global wd % desired angular position

vd = 10;
wd = 2*pi/3;

%% plot dynamics
t = 20; % how far to take data from the IC
ts = 500; % sample size
tspan = linspace(0,t,ts); % discretized time values to sample
tol = 1e-10; % universal tolerance
options = odeset('RelTol',tol,'AbsTol',[tol tol tol]); % tolerances
init_cond = [-vd, pi/2-wd, 0]; % initial conditions
[T,X] = ode45(@interceptor45,tspan,init_cond,options);
figure(1)
plot(tspan,X)
legend('x1 - forward velocity','x2 - angular position','x3 - angular velocity')
grid on;

%% define initial position and orientation

x = 0; % initial x position
y = 0; % initial y position
phi = 0; % initial angle added onto desired angle

%% iterate through time
for n = 2:1:ts
    % X(n,1) is forward velocity
    % X(n,2) is angular position
    % X(n,3) is angular velocity
    
    % get change in time
    dt = T(n) - T(n-1);
    
    % update angular orientation
    phi = X(n,2) + wd;
    
    % update position
    x = x + (X(n,1)+vd)*cos(phi)*dt;
    y = y + (X(n,1)+vd)*sin(phi)*dt;
    
    % draw the interceptor on the x-y plane
    figure(2)
    plot(x,y,'ko')
    w = 50; % window size
    axis([-w w 0 w])
    grid on
    hold on
    
    % draw the target
    figure(2)
    plot(xt,yt,'ro')
    
    % draw the desired time to impact
    timer = sprintf('%d',int8(t_hit));
    text(1.1*xt,1.1*yt,timer)
    t_hit = t_hit - dt;
    hold off
    
    % capture frame for movie
    F(n-1) = getframe;
    % movie(F) % to play movie
end

%% functions
function dx = interceptor45(~,x)

global vd
global wd

% set up dynamics for interceptor
A = [0 0 0 ;
     0 0 1 ;
     0 0 0];
B = [1 0 ;
     0 0 ;
     0 1];

% define control law
u = zeros(2,1);
r = [vd; % desired forward velocity
     wd; % desired angular position
     0]; % desired angular velocity
p = [-1 -2 -3];
k = place(A,B,p);
u = -k*x;

% define results
dx = A*(x-r) + B*u + A*r;
end