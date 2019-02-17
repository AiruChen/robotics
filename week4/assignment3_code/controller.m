function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

%% Thrust
%kdr = [0;0;300];          %kdx; kdy; kdz
kdr = [10;10;10];          %kdx; kdy; kdz
kpr = [500;500;500];         %kpx; kpy; kpz 300; 300; 300
kdangle = [1;1;1];      %kdphi; kdtheta; kdpsi
kpangle = [200;200;200];   %kpphi; kdtheta; kdpsi

% Calculate r_ddot_command1,2,3
r_ddot_command = des_state.acc + kdr.*(des_state.vel - state.vel) + kpr.*(des_state.pos - state.pos);
% u1 = m(g+r_ddot_command_3)
F = params.mass * (params.gravity + r_ddot_command(3));

%% Moment
phi_command = (r_ddot_command(1)*sin(des_state.yaw) - r_ddot_command(2)*cos(des_state.yaw))/params.gravity;
theta_command = (r_ddot_command(1)*cos(des_state.yaw) + r_ddot_command(2)*sin(des_state.yaw))/params.gravity;
psi_command = des_state.yaw;
M = [ kpangle(1)*(phi_command - state.rot(1)) + kdangle(1)*(phi_command - state.omega(1));
    kpangle(2)*(theta_command - state.rot(2)) + kdangle(2)*(theta_command - state.omega(2));
    kpangle(3)*(psi_command - state.rot(3)) + kdangle(3)*(psi_command - state.omega(3));];

% =================== Your code ends here ===================
% state.vel
% des_state.vel
% state.rot(3)
% des_state.yaw
end
