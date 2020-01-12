clc; clear; close all;

%% Define target
% Target is defined as a collision point in both space and time

% Target collision location in cartesian coordinates
yt = [10 0 40 0]';

% Collision time
t_hit = 5;

%% Missile dynamics

% Set up high level dynamics for interceptor
angular = AngularDynamics;

cart = CartesianDynamics;

% Initial Conditions
vi = 5;
wi = pi/2;

angular.x = [vi wi 0]';
cart.x = [0 vi*cos(wi) 0 vi*sin(wi)]';

%% Custom iterative solver
dt = 0.02; % timestep size

end_time = t_hit; % Can be changed to show behavior after collision
tspan = 0:dt:end_time;

steps = size(tspan,2);
tr = t_hit; % time remaining to impact

X = zeros(steps,size(angular.x,1));
U = zeros(steps,2);

for i = 1:length(tspan)
    %% Define angular control law
    
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
    
    %% Define cartesian control law
    
    r2 = [cart.x(1) ;
          angular.x(1)*cos(angular.x(2)) ;
          cart.x(3) ;
          angular.x(1)*sin(angular.x(2))];
      
    [dx2,u2] = Control(cart,r2);
    dx2 = dx2-[0 0 0 1.5]'; % Gravity
    cart.x = cart.x+dx2*dt;
    
    %% Model the target
    
    % Uncomment to model movement in the target
%     yt(1) = yt(1)-5*dt;
    
    % Uncomment to model noise in the target
%     yt(1) = yt(1)+(20*rand-10)*dt;
%     yt(3) = yt(3)+(20*rand-10)*dt;
    
    %% Draw the animation
    
    % Plot the missile
    figure(1)
    plot(cart.x(1),cart.x(3),'ko')
    
    % Set the window
    w = 50; % window size
    axis([-w w 0 w])
    title('Position of the Point Mass and Target')
    xlabel('Horizontal Position')
    ylabel('Vertical Position')
    
    % Draw the distance and direction information text
    dist_text = sprintf('Distance: %f',d);
    dir_text = sprintf('Angular Offset: %f', wd);
    text(cart.x(1)+5, cart.x(3)-5, dist_text);
    text(cart.x(1)+5, cart.x(3)-10, dir_text);
    
    % Draw the desired time to impact
    tr = t_hit - tspan(i);
    timer = sprintf('Time Remaining: %.01f',tr);
    text(cart.x(1)+5,cart.x(3),timer)
    
    % Draw the target
    grid on
    hold on
    plot(yt(1),yt(3),'ro')
    hold off
    
    % Capture the frame to render the video
%     F(i) = getframe;
    
end

%% Plot the system behavior

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

%% Write the video

% v = VideoWriter('Interceptor.avi');
% v.FrameRate = 50;
% v.Quality = 95;
% open(v)
% for j=1:(i-1)
%     writeVideo(v, F(j))
% end
% close(v)