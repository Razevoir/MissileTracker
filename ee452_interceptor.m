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

% Set up cartesian plane dynamical model
cart.A = [0 1 0 0 ;
     0 0 0 0 ;
     0 0 0 1 ;
     0 0 0 0];
cart.B = [0 0 ;
     1 0 ;
     0 0 ;
     0 1];
 
cart.p = [-20 -25 -30 -35];
cart.k = place(cart.A,cart.B,cart.p);
cart.wrap = [0 0 0 0];
cart.limits = [-20 20 ;
               -20 20];

% Cartesian coordinates
% x(1) => x position
% x(2) => x velocity
% x(3) => y position
% x(4) => y velocity
cart.x = [0 ;
     angular.vi*cos(angular.wi) ;
     0 ;
     angular.vi*sin(angular.wi)];

%% Custom iterative solver
dt = 0.02; % timestep size
end_time = t_hit; % Can be changed to show behavior after collision
tspan = 0:dt:end_time;
steps = size(tspan,2);
tr = t_hit; % time remaining to impact

X = zeros(steps,size(angular.x,1));
U = zeros(steps,2);

% Used to capture frames
i = 1;

for n = 0:dt:end_time
    % define control law
    e = yt-cart.x;
    
    d=sqrt(e(1)^2+e(3)^2);
    vd = d/tr;
    
    wd = atan2(e(3),e(1));
    
    r = [vd; % desired forward velocity
         wd; % desired angular position
         0]; % desired angular velocity
     
    [dx,u] = Control(angular,r);
    angular.x = angular.x+dx*dt;
    
    % Store the output to plot later
    U(i,:) = u';
    X(i,:) = angular.x';
    
    % Define control law for cartesian coordinates
    r2 = [cart.x(1) ;
          angular.x(1)*cos(angular.x(2)) ;
          cart.x(3) ;
          angular.x(1)*sin(angular.x(2))];
      
    [dx2,u2] = Control(cart,r2);
    dx2 = dx2-[0 0 0 1.5]'; % Gravity
    cart.x = cart.x+dx2*dt;
    
    % Uncomment to model movement in the target
%     yt(1) = yt(1)-5*dt;
    
    % Uncomment to model noise in the target
%     yt(1) = yt(1)+(20*rand-10)*dt;
%     yt(3) = yt(3)+(20*rand-10)*dt;
    
    % draw the interceptor on the x-y plane
    figure(1)
    plot(cart.x(1),cart.x(3),'ko')
    w = 50; % window size
    axis([-w w 0 w])
    title('Position of the Point Mass and Target')
    xlabel('Horizontal Position')
    ylabel('Vertical Position')
    
    % Draw the distance and direction
    dist_text = sprintf('Distance: %f',d);
    dir_text = sprintf('Angular Offset: %f', wd);
    text(cart.x(1)+5, cart.x(3)-5, dist_text);
    text(cart.x(1)+5, cart.x(3)-10, dir_text);
    
    grid on
    hold on
    
    % draw the target
    plot(yt(1),yt(3),'ro')
    
    % draw the desired time to impact
    tr = t_hit - n;
    timer = sprintf('Time Remaining: %.01f',tr);
    text(1.1*cart.x(1),1.1*cart.x(3),timer)
    
    hold off
    
    % capture frame for movie
    i = i+1;
    %F(i-1) = getframe;
    
end

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