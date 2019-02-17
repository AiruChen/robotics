function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

%% FILL IN YOUR CODE HERE

% u = params.mass * params.gravity % For hover u = mg

persistent s_last; % store last time's state
if isempty(s_last) % init the last time state
    s_last = s;
end

% error can get from the differences between current and destination
error = s_des - s; 

dz = s(1) - s_last(1); % unuse
ddz = s(2) - s_last(2); % the acceleration can get from different velocity between current and last state.

kp = 400; % Proporgation 
kv = 50; % Derivative

% u = m*(ddz + kp*e + kv*de + g)
u = params.mass * (ddz + kp*error(1) + kv*error(2) + params.gravity); 

s_last = s; % store the current into last
end

