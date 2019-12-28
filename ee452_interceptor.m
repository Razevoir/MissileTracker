clc; clear all; close all;

%% define target
% can define target as either desired velocity and angle for the missile...
% ... or as a position in 2d space to try and hit during flight

% target location in cartesian coordinates
xt = 10;
yt = 40;

% time to hit the target
t_hit = 5;

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
 
p = [-20 -25 -30]; % desired pole locations
k = place(A,B,p); % state feedback gains

X = [vd, pi/2, 0]; % initial conditions
x = X.';

dt = 0.02; % timestep size
end_time = t_hit;
tr = t_hit; % time remaining to impact

U = zeros(1,2);

i = 1;

for n = dt:dt:end_time
    i = i+1;
    % define control law
    d = sqrt((xt-xm(i-1))^2+(yt-ym(i-1))^2);
    vd = d/tr;
    w = atan2(yt-ym(i-1), xt-xm(i-1));
    wd = wrapToPi(w);
    r = [vd; % desired forward velocity
        wd; % desired angular position
        0]; % desired angular velocity
    xh = x-r; % error states
    
    % calculate control effort
    u = -k*xh; % control input
    U = [U;u']; % control input matrix for plotting
    
    % define results
    dx = A*x+B*u; % change in states
    x = x + dx*dt; % update old states to new states
    X = [X; x.']; % add new states to memory to be plotted
    
    xm(i) = xm(i-1)+x(1)*cos(x(2))*dt; % x-coordinate of the missile
    ym(i) = ym(i-1)+x(1)*sin(x(2))*dt; % y-coordinate of the missile
    
    % X(n,1) is forward velocity
    % X(n,2) is angular position
    % X(n,3) is angular velocity
    
    % Uncomment to model movement in the target
    %xt = xt-5*dt;
    
    % Uncomment to model noise in the target
    xt = xt+(20*rand-10)*dt;
    yt = yt+(20*rand-10)*dt;
    
    % draw the interceptor on the x-y plane
    figure(1)
    plot(xm(i),ym(i),'ko')
    w = 50; % window size
    axis([-w w 0 w])
    
    % Draw the distance and direction
    dist_text = sprintf('%f',d);
    dir_text = sprintf('%f', wd);
    text(xm(i)+5, ym(i)-5, dist_text);
    text(xm(i)+5, ym(i)-10, dir_text);
    
    grid on
    hold on
    
    % draw the target
    % figure(1)
    plot(xt,yt,'ro')
    
    % draw the desired time to impact
    tr = tr - dt;
    timer = sprintf('%.01f',tr);
    text(1.1*xt,1.1*yt,timer)
    
    hold off
    
    % capture frame for movie
    F(i-1) = getframe;
    % movie(F) % to play movie
    
end
tspan = 0:dt:end_time;

% Plot states VS time
figure(2)
plot(tspan,X)
legend('x1 - forward velocity','x2 - angular position','x3 - angular velocity')
grid on

% plot controller effort VS time
figure(3)
plot(tspan,U)
legend('u1 - forward thrusters','u2 - angular thrusters')
grid on

% v = VideoWriter('Interceptor.avi');
% v.FrameRate = 50;
% v.Quality = 95;
% open(v)
% for j=1:(i-1)
%     writeVideo(v, F(j))
% end
% close(v)