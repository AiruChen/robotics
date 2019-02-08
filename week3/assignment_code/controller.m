function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
% keyboard;
u1 = 0; % params.mass*params.gravity;
u2 = 0;

% FILL IN YOUR CODE HERE
kd_z = 20;
kp_z = 100;
kd_y = 20;
kp_y = 100;
kd_phi = 10;
kp_phi = 80;

e.pos = des_state.pos - state.pos;
e.vel = des_state.vel - state.vel;

u1 = params.mass*(params.gravity + des_state.acc(2) + kd_z*e.vel(2) + kp_z*e.pos(2));
phi_c = -(des_state.acc(1) + kd_y*e.vel(1) + kp_y*e.pos(1))/ params.gravity;
u2 = kd_phi*(-state.omega) + kp_phi*(phi_c - state.rot);


end

