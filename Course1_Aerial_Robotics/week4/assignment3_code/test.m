clear all
close all
clc
waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';
POS = zeros(3,1);
desired_state.pos = zeros(3,1);
desired_state.vel = zeros(3,1);
desired_state.acc = zeros(3,1);
desired_state.yaw = 0;
desired_state.yawdot = 0;
scale = 0;
for t = 0:0.1:15
% waypoints0 traj_time d0 coffx coffy coffz
if t == 0
    
    [coffx,A_x,b_x] = getCoff(waypoints, 'x');
    [coffy,A_y,b_y] = getCoff(waypoints, 'y');
    [coffz,A_z,b_z] = getCoff(waypoints, 'z');
    
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
        %{
        pos = [coffx( 1+8*(t_index-1) : 8*t_index)'; coffy( 1+8*(t_index-1) : 8*t_index)'; coffz( 1+8*(t_index-1) : 8*t_index)']...
            *[1 scale scale^2 scale^3 scale^4 scale^5 scale^6 scale^7]'; 
        vel = [coffx( 1+8*(t_index-1) : 8*t_index)'; coffy( 1+8*(t_index-1) : 8*t_index)'; coffz( 1+8*(t_index-1) : 8*t_index)']...
            *[0 1 2*scale 3*scale^2 4*scale^3 5*scale^4 6*scale^5 7*scale^6]'; 
        acc = [coffx( 1+8*(t_index-1) : 8*t_index)'; coffy( 1+8*(t_index-1) : 8*t_index)'; coffz( 1+8*(t_index-1) : 8*t_index)']...
            *[0 0 2 6*scale 12*scale^2 20*scale^3 30*scale^4 42*scale^5 ]'; 
        %}
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
%{
desired_state.pos;
plot3(desired_state.pos(1),desired_state.pos(2),desired_state.pos(3),'ro')
grid on
xlabel('x')
ylabel('y')
zlabel('z')
drawnow
hold on
pause(0.1)
%}
subplot(2,2,1)
plot(t,desired_state.pos(1),'ro')
hold on
plot(t,desired_state.pos(2),'go')
hold on
plot(t,desired_state.pos(3),'bo')
title('pos')
grid on

subplot(2,2,2)
plot(t,desired_state.vel(1),'ro')
hold on
plot(t,desired_state.vel(2),'go')
hold on
plot(t,desired_state.vel(3),'bo')
title('vel')
grid on

subplot(2,2,3)
plot(t,desired_state.acc(1),'ro')
hold on
plot(t,desired_state.acc(2),'go')
hold on
plot(t,desired_state.acc(3),'bo')
title('acc')
grid on

subplot(2,2,4)
plot(t,scale,'blo')
hold on
grid on

end
