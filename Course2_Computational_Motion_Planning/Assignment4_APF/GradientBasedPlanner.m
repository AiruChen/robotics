function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
robot_pos = start_coords;
route = robot_pos;

i = 1;
dis_2_end = norm(end_coords - start_coords);

while(i<max_its && dis_2_end > 2)
    
    V = [gx(round(robot_pos(2)), round(robot_pos(1))) gy(round(robot_pos(2)), round(robot_pos(1)))];
    V = V/norm(V);
    
    robot_pos = robot_pos + V;
    
    dis_2_end = norm(end_coords - robot_pos);
    
    route = double([route; robot_pos]);
end

% *******************************************************************
end
