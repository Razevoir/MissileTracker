clc; clear; close all;

%% Define target
% Target is defined as a collision point in both space and time

% Target collision location in cartesian coordinates
yt = [10 0 40 0]';

% Collision time
t_hit = 5;

%% Missile dynamics

% Set up high level dynamics for interceptor
%angular = AngularDynamics;

cart = CartesianDynamics;

% Initial Conditions
vi = 5;
wi = pi/2;

%angular.x = [vi wi 0]';
cart.x = [0 vi*cos(wi) 0 vi*sin(wi)]';

%% Custom iterative solver
dt = 0.02; % timestep size

end_time = t_hit; % Can be changed to show behavior after collision
tspan = 0:dt:end_time;

steps = size(tspan,2);
tr = t_hit; % time remaining to impact

X = zeros(steps,size(cart.x,1));
U = zeros(steps,2);

for i = 1:length(tspan)
    %% Define cartesian control law
    % Establish the reference values
    e = yt-cart.x;
    
    % Determine the velocity needed to reach the target at tr=0
    d=sqrt(e(1)^2+e(3)^2);
    vd = d/tr;
    % Find the direction to accelerate
    wd = atan2(e(3),e(1));
    % Build the reference vector
    r2 = [cart.x(1)  ;
          vd*cos(wd) ;
          cart.x(3)  ;
          vd*sin(wd)];

    % Determine the controller input
    xh = cart.x-r2;
    u = -cart.k*xh;
    
    % Find the forward and orthogonal vectors of motion
    vf = [cart.x(2) cart.x(4)]';
    vo = [vf(2) -vf(1)]';
    % Normalize
    vf = vf/norm(vf);
    vo = vo/norm(vo);
    
    % Project control vector onto the relative missile vectors
    ff = dot(u,vf);
    of = dot(u,vo);
    % Limit the magnitude of the forward and orthogonal forces
    ff = median([-10 ff 10]);
    of = median([-2 of 2]);
    % Set u to the new limited values
    u = ff*vf+of*vo;
    
    % Find the state differential
    dx2 = cart.A*cart.x+cart.B*u;

    % External forces (gravity, etc.)
    dx2 = dx2-[0 0 0 1.5]';
    
    % Apply law
    cart.x = cart.x+dx2*dt;
    
    % Plot effort and states
    U(i,:) = [ff of];
    X(i,:) = cart.x';
    
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
legend('x1 - Horizontal Position','x2 - Horizontal Velocity','x3 - Vertical Position','x4 - Vertical Velocity')
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