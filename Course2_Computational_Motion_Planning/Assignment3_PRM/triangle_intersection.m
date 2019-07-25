function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag = true;

P1_tri = [P1; P1];
P2_tri = [P2; P2];

for i=1:3
    d1 = det(P1_tri(i,:), P1_tri(i+1,:), P1_tri(i+2,:));
    d2 = det(P1_tri(i,:), P1_tri(i+1,:), P2_tri(1,:));
    d3 = det(P1_tri(i,:), P1_tri(i+1,:), P2_tri(2,:));
    d4 = det(P1_tri(i,:), P1_tri(i+1,:), P2_tri(3,:));
    if ((d1>0 && d2<0 && d3<0 && d4<0) || (d1<0 && d2>0 && d3>0 && d4>0))
        flag = false;
        break;
    end
end
for i=1:3
    d1 = det(P2_tri(i,:), P2_tri(i+1,:), P2_tri(i+2,:));
    d2 = det(P2_tri(i,:), P2_tri(i+1,:), P1_tri(1,:));
    d3 = det(P2_tri(i,:), P2_tri(i+1,:), P1_tri(2,:));
    d4 = det(P2_tri(i,:), P2_tri(i+1,:), P1_tri(3,:));
    if ((d1>0 && d2<0 && d3<0 && d4<0) || (d1<0 && d2>0 && d3>0 && d4>0))
        flag = false;
        break;
    end
end
% *******************************************************************
end

function d = det(A, B, P)
    d = (P(1)-A(1))*(B(2)-A(2)) - (P(2)-A(2))*(B(1)-A(1));
end