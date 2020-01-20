function [out] = inverse_kinematics(noap)
nx = noap(1, 1);
ny = noap(2, 1);
nz = noap(3, 1);
ox = noap(1, 2);
oy = noap(2, 2);
oz = noap(3, 2);
ax = noap(1, 3);
ay = noap(2, 3);
az = noap(3, 3);
px = noap(1, 4);
py = noap(2, 4);
pz = noap(3, 4);
d2 = 16.1925;
theta1 = zeros(2, 1);
theta2 = zeros(4, 1);
d3 = zeros(4, 1);
theta4 = zeros(4, 1);
theta5 = zeros(4, 1);
theta6 = zeros(4, 1);
c1 = zeros(2, 1);
s1 = zeros(2, 1);
c2 = zeros(4, 1);
s2 = zeros(4, 1);
c4 = zeros(4, 1);
s4 = zeros(4, 1);


% ============ theta1 ============ %
r=sqrt(px^2+py^2);
theta1(1) = atan2(py,px)-atan2(d2,sqrt(r^2-d2^2));
theta1(2) = atan2(py,px)-atan2(d2,-sqrt(r^2-d2^2));
for i = 1:2
    c1(i) = cos(theta1(i));
    s1(i) = sin(theta1(i));
end
% ------- theta1 has 2 solutions ------- % 



% ============ theta2 ============ %
theta2(1) = atan2(c1(1)*px+s1(1)*py,pz);
theta2(2) = atan2(c1(2)*px+s1(2)*py,pz);
theta2(3) = atan2(c1(1)*px+s1(1)*py,pz);
theta2(4) = atan2(c1(2)*px+s1(2)*py,pz);
for i = 1:4
    c2(i) = cos(theta2(i));
    s2(i) = sin(theta2(i));
end
% ------- theta2 has 2 solutions ------- % 



% ============ d3 ============ %
d3(1) = s2(1)*(c1(1)*px + s1(1)*py) + c2(1)*pz;
d3(2) = s2(2)*(c1(2)*px + s1(2)*py) + c2(2)*pz;
d3(3) = s2(3)*(c1(1)*px + s1(1)*py) + c2(3)*pz;
d3(4) = s2(4)*(c1(2)*px + s1(2)*py) + c2(4)*pz;
% ------- d3 has 1 solutions ------- % 



% ============ theta4 ============ %
theta4(1) = atan2(-s1(1)*ax+c1(1)*ay,c2(1)*(c1(1)*ax+s1(1)*ay)-s2(1)*az);
theta4(2) = atan2(-s1(2)*ax+c1(2)*ay,c2(2)*(c1(2)*ax+s1(2)*ay)-s2(2)*az);
theta4(3) = atan2(-(-s1(1)*ax+c1(1)*ay),-(c2(1)*(c1(1)*ax+s1(1)*ay)-s2(1)*az));
theta4(4) = atan2(-(-s1(2)*ax+c1(2)*ay),-(c2(2)*(c1(2)*ax+s1(2)*ay)-s2(2)*az));
for i = 1:4
    c4(i) = cos(theta4(i));
    s4(i) = sin(theta4(i));
end
% ------- theta4 has 4 solutions ------- % 



% ============ theta6 ============ %
theta6(1) = atan2(c1(1)*s2(1)*ox+s1(1)*s2(1)*oy+c2(1)*oz,-(c1(1)*s2(1)*nx+s1(1)*s2(1)*ny+c2(1)*nz));
theta6(2) = atan2(c1(2)*s2(2)*ox+s1(2)*s2(2)*oy+c2(2)*oz,-(c1(2)*s2(2)*nx+s1(2)*s2(2)*ny+c2(2)*nz));
theta6(3) = atan2(-(c1(1)*s2(1)*ox+s1(1)*s2(1)*oy+c2(1)*oz),-(-(c1(1)*s2(1)*nx+s1(1)*s2(1)*ny+c2(1)*nz)));
theta6(4) = atan2(-(c1(2)*s2(2)*ox+s1(2)*s2(2)*oy+c2(2)*oz),-(-(c1(2)*s2(2)*nx+s1(2)*s2(2)*ny+c2(2)*nz)));
% ------- theta6 has 4 solutions ------- % 



% ============ theta5 ============ %
theta5(1) = atan2(c2(1)*c4(1)*(c1(1)*ax+s1(1)*ay)-s2(1)*c4(1)*az-s1(1)*s4(1)*ax+c1(1)*s4(1)*ay,s2(1)*(c1(1)*ax+s1(1)*ay)+c2(1)*az);
theta5(2) = atan2(c2(2)*c4(2)*(c1(2)*ax+s1(2)*ay)-s2(2)*c4(2)*az-s1(2)*s4(2)*ax+c1(2)*s4(2)*ay,s2(2)*(c1(2)*ax+s1(2)*ay)+c2(2)*az);
theta5(3) = atan2(-(c2(1)*c4(1)*(c1(1)*ax+s1(1)*ay)-s2(1)*c4(1)*az-s1(1)*s4(1)*ax+c1(1)*s4(1)*ay),(s2(1)*(c1(1)*ax+s1(1)*ay)+c2(1)*az));
theta5(4) = atan2(-(c2(2)*c4(2)*(c1(2)*ax+s1(2)*ay)-s2(2)*c4(2)*az-s1(2)*s4(2)*ax+c1(2)*s4(2)*ay),(s2(2)*(c1(2)*ax+s1(2)*ay)+c2(2)*az));
% ------- theta5 has 4 solutions ------- % 



% ============ output joint variables ============ %
out = [theta1(1) theta2(1) d3(1)*pi/180 theta4(1) theta5(1) theta6(1);
         theta1(2) theta2(2) d3(2)*pi/180 theta4(2) theta5(2) theta6(2);
         theta1(1) theta2(3) d3(1)*pi/180 theta4(3) theta5(3) theta6(3);
         theta1(2) theta2(4) d3(2)*pi/180 theta4(4) theta5(4) theta6(4);
         ]*180/pi;
     