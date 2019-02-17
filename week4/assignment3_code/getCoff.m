function [coff, A, b] = getCoff(waypoints, xyz)
n = size(waypoints',1)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1,8*n);

% --------------------------------------------

% YOUR CODE HERE 
% Fill A and b matices with values using loops

% P1(0), P2(0), P3(0), P4(0)
if xyz == 'x'
    xyz_coff = 1;
elseif xyz == 'y'
    xyz_coff = 2;
elseif xyz == 'z'
    xyz_coff = 3; 
end

for i=1:n
    b(1,i) = waypoints(xyz_coff,i);
end

row = 1;
for i=1:n
    A(row,8*(i-1)+1:8*i) = polyT(8,0,0); 
    row = row + 1;
end
% P1(1), P2(1), P3(1), p4(1)
for i = 1:n
    b(1, row-1+i) = waypoints(xyz_coff, i+1);
end
for i = 1:n
    A(row, 8*(i-1)+1:8*i) = polyT(8,0,1);
    row = row + 1;
end
% P1_dot(0), p1_ddot(0), p1_dddot(0)
for i = 1:3
    b(1, row-1+i) = 0;
end
for i = 1:3
    A(row, 1:8) = polyT(8, i, 0);
    row = row + 1;
end
% P4_dot(1), p4_ddot(1), p4_dddot(1)
for i = 1:3
    b(1, row-1+i) = 0;
end
for i = 1:3
    A(row, 8*(4-1)+1:8*4) = polyT(8, i, 1);
    row = row + 1;
end
%{
P1_dot(1)-P2_dot(0)=0; P2_dot(1)-P3_dot(0)=0; P3_dot(1)-P4_dot(0)=0;
P1_ddot(1)-P2_ddot(0)=0; P2_ddot(1)-P3_ddot(0)=0; P3_ddot(1)-P4_ddot(0)=0;
...;
P1_d(6)ot(1)-P2_d(6)ot(0)=0; P2_d(6)ot(1)-P3_d(6)ot(0)=0; P3_d(6)ot(1)-P4_d(6)ot(0)=0;
%}
for i = 1:18
    b(1, row-1+i) = 0;
end
for i = 1:6
    for j = 1:3
        A(row, 8*(j-1)+1:8*(j+1)) = [polyT(8, i, 1) -polyT(8,i,0)];
        row = row + 1;
    end
end
% --------------------------------------------
coff = inv(A)*b';
end

