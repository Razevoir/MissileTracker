clc; clear all; close all;

%% define target
% can define target as either desired velocity and angle for the missile...
% ... or as a position in 2d space to try and hit during flight

% target location in cartesian coordinates
xt = 10;
yt = 40;

% time to hit the target
t_hit = 10;

% missile location in cartesian coordinates
xm = 0;
ym = 0;

vd = 5;
wd = 2*pi/3;

%% Iterative Solver to replace ode45

% set up dynamics for interceptor
A = [0 0 0 ;
     0 0 1 ;
     0 0 0];
B = [1 0 ;
     0 0 ;
     0 1];
X = [vd, pi/2, 0]; % initial conditions
x = X.';
dt = 0.05; % timestep size
end_time = t_hit;
i = 1;
for n = dt:dt:end_time
    i = i+1;
    % define control law
    %vd = 10;%sqrt((xt-xm)^2+(yt-ym)^2);
    wd = atan2(yt-ym(i-1), xt-xm(i-1));
    r = [vd; % desired forward velocity
        wd; % desired angular position
        0]; % desired angular velocity
    p = [-1 -2 -3];
    k = place(A,B,p);
    xh = x-r;
    xh(2) = wrapToPi(xh(2));
    
    % define results
    dx = (A-B*k)*(xh);
    x = x + dx*dt;
    X = [X; x.'];
    
    xm(i) = xm(i-1)+x(1)*cos(x(2))*dt;
    ym(i) = ym(i-1)+x(1)*sin(x(2))*dt;
    
end
tspan = 0:dt:end_time;

figure(1)
plot(tspan,X)
legend('x1 - forward velocity','x2 - angular position','x3 - angular velocity')
grid on;


%% define initial position and orientation for iteration

phi = 0; % initial angle added onto desired angle

%% iterate ode45 results through time
tr = t_hit; % time remaining to impact
for n = 2:1:size(tspan')
    % X(n,1) is forward velocity
    % X(n,2) is angular position
    % X(n,3) is angular velocity
    
    % Calculate the trajectory to the target
    distance = sqrt((xt-xm(n))^2+(yt-ym(n))^2);
    direction = atan2(yt-ym(n), xt-xm(n));
    
    % draw the interceptor on the x-y plane
    figure(3)
    plot(xm(n),ym(n),'ko')
    w = 50; % window size
    axis([-w w 0 w])
    
    % Draw the distance and direction
    dist_text = sprintf('%f',distance);
    dir_text = sprintf('%f', direction);
    text(xm(n)+5, ym(n)-5, dist_text);
    text(xm(n)+5, ym(n)-10, dir_text);
    
    grid on
    hold on
    
    % draw the target
    %figure(3)
    plot(xt,yt,'ro')
    
    % draw the desired time to impact
    timer = sprintf('%d',int8(tr));
    text(1.1*xt,1.1*yt,timer)
    tr = tr - dt;
    
    hold off
    
    % capture frame for movie
    F(n-1) = getframe;
    % movie(F) % to play movie
end