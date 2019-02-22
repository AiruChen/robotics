function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.
%{
persistent waypoints0 traj_time d0
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2); % 2 = 1/speed = 1/0.5; cal the spending time for every traj
    traj_time = [0, cumsum(d0)]; % cumsum: cultivate sum [1 2 3 4 5]=>[1 3 6 10 15]
    waypoints0 = waypoints;
else
    if(t > traj_time(end))  % if the time past the total needing time
        t = traj_time(end); % t stop
    end
    t_index = find(traj_time >= t,1);   % find the next goal waypoints number

    if(t_index > 1) % if the next goal is not the first one
        t = t - traj_time(t_index-1);   % then reset the time for each goal, current_time - last_needing_time
    end
    if(t == 0)  % if just the beginning
        desired_state.pos = waypoints0(:,1);    % init the des_state.pos
    else
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%}


%% Fill in your code here
persistent waypoints0 traj_time d0 coffx coffy coffz
if nargin > 2
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    coffx = getCoff(waypoints, 'x');
    coffy = getCoff(waypoints, 'y');
    coffz = getCoff(waypoints, 'z');
    
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
else
    if(t > traj_time(end))
        t = traj_time(end)- 0.0001;
    end
    t_index = find(traj_time>t,1) - 1;
    t_index = max(t_index,1);
    scale = (t - traj_time(t_index)) / d0(t_index);
    if (t == 0)
        pos = waypoints0(:,1);% init the des_state.pos
        vel = zeros(3,1);
        acc = zeros(3,1);
    else
        pos = [coffx( 1+8*(t_index-1) : 8*t_index)'; coffy( 1+8*(t_index-1) : 8*t_index)'; coffz( 1+8*(t_index-1) : 8*t_index)']...
            *polyT(8, 0, scale)'; 
        vel = [coffx( 1+8*(t_index-1) : 8*t_index)'; coffy( 1+8*(t_index-1) : 8*t_index)'; coffz( 1+8*(t_index-1) : 8*t_index)']...
            *polyT(8, 1, scale)' .*(1/d0(t_index)); 
        acc = [coffx( 1+8*(t_index-1) : 8*t_index)'; coffy( 1+8*(t_index-1) : 8*t_index)'; coffz( 1+8*(t_index-1) : 8*t_index)']...
            *polyT(8, 2, scale)' .*(1/d0(t_index)^2);
    end 
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%}

end
