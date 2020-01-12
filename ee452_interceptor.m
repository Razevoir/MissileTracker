clc; clear all; close all;

%% define target
% can define target as either desired velocity and angle for the missile...
% ... or as a position in 2d space to try and hit during flight

% target location in cartesian coordinates
yt=[10 0 40 0]';

% time to hit the target
t_hit = 5;

%% Missile dynamics

% Set up high level dynamics for interceptor
angular = AngularDynamics;
X = angular.x';

% Set up cartesian plane dynamical model
C = [0 1 0 0 ;
     0 0 0 0 ;
     0 0 0 1 ;
     0 0 0 0];
D = [0 0 ;
     1 0 ;
     0 0 ;
     0 1];
 
 p2 = [-20 -25 -30 -35];
 k2 = place(C,D,p2);

% Cartesian coordinates
% y(1) => x position
% y(2) => x velocity
% y(3) => y position
% y(4) => y velocity
y = [0 ;
     angular.vi*cos(angular.wi) ;
     0 ;
     angular.vi*sin(angular.wi)];

%% Custom iterative solver
dt = 0.02; % timestep size
end_time = t_hit;
tr = t_hit; % time remaining to impact

U = zeros(1,2);

% Used to capture frames
i = 1;

for n = dt:dt:end_time
    % define control law
    e = yt-y;
    
    d=sqrt(e(1)^2+e(3)^2);
    vd = d/tr;
    
    wd = atan2(e(3),e(1));
    
    r = [vd; % desired forward velocity
         wd; % desired angular position
         0]; % desired angular velocity
     
    [dx,u] = AngularControl(angular.A,angular.B,angular.x,angular.k,r,angular.limits);
    angular.x = angular.x+dx*dt;
    
    % Store the output to plot later
    U = [U;u'];
    X = [X;angular.x'];
    
    % Define control law for cartesian coordinates
    r2 = [y(1) ;
          angular.x(1)*cos(angular.x(2)) ;
          y(3) ;
          angular.x(1)*sin(angular.x(2))];
    yh = y-r2;
    v = -k2*yh;
    dy = C*y+D*v;
    dy = dy-[0 0 0 1.5]'; % Gravity
    y = y+dy*dt;
    
    % Uncomment to model movement in the target
%     yt(1) = yt(1)-5*dt;
    
    % Uncomment to model noise in the target
%     yt(1) = yt(1)+(20*rand-10)*dt;
%     yt(3) = yt(3)+(20*rand-10)*dt;
    
    % draw the interceptor on the x-y plane
    figure(1)
    plot(y(1),y(3),'ko')
    w = 50; % window size
    axis([-w w 0 w])
    title('Position of the Point Mass and Target')
    xlabel('Horizontal Position')
    ylabel('Vertical Position')
    
    % Draw the distance and direction
    dist_text = sprintf('Distance: %f',d);
    dir_text = sprintf('Angular Offset: %f', wd);
    text(y(1)+5, y(3)-5, dist_text);
    text(y(1)+5, y(3)-10, dir_text);
    
    grid on
    hold on
    
    % draw the target
    % figure(1)
    plot(yt(1),yt(3),'ro')
    
    % draw the desired time to impact
    tr = tr - dt;
    timer = sprintf('Time Remaining: %.01f',tr);
    text(1.1*y(1),1.1*y(3),timer)
    
    hold off
    
    % capture frame for movie
    i = i+1;
    F(i-1) = getframe;
    % movie(F) % to play movie
    
end
tspan = 0:dt:end_time;

% Plot states VS time
figure(2)
plot(tspan,X)
title('System States vs Time')
xlabel('Time')
ylabel('Magnitude')
legend('x1 - forward velocity','x2 - angular position','x3 - angular velocity')
grid on

% plot controller effort VS time
figure(3)
plot(tspan,U)
title('Controller Effort vs Time')
xlabel('Time')
ylabel('Magnitude')
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