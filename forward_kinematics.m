% ============ input = [theta1 theta2 d3 theta4 theta5 theta6] ============ %
function [out] = forward_kinematics(theta_input)
c = zeros(6, 1);                            % array to save cos
s = zeros(6, 1);                            % array to save sin

for i = 1:6
   c(i) = cos(pi/180*theta_input(i));
   s(i) = sin(pi/180*theta_input(i));
end

% ========================= kinetic table ========================= %
d2 = 16.1925;
d3 = theta_input(3);

A1 = [c(1) 0 -s(1) 0; s(1) 0 c(1) 0; 0 -1 0 0; 0 0 0 1];
A2 = [c(2) 0 s(2) 0; s(2) 0 -c(2) 0; 0 1 0 d2; 0 0 0 1];
A3 = [1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
A4 = [c(4) 0 -s(4) 0; s(4) 0 c(4) 0; 0 -1 0 0; 0 0 0 1];
A5 = [c(5) 0 s(5) 0; s(5) 0 -c(5) 0; 0 1 0 0; 0 0 0 1];
A6 = [c(6) -s(6) 0 0; s(6) c(6) 0 0; 0 0 1 0; 0 0 0 1];

T6 = A1*A2*A3*A4*A5*A6;

% ============ compute the x , y , z , phi , theta , phi ============ %

%T6 = [0.0805 -0.3572 -0.9306 -7.9883; 0.3571 0.8819 -0.3076 8.5915; 0.9306 -0.3076 0.1986 -1.7365; 0 0 0 1];

x = T6(1,4);
y = T6(2,4);
z = T6(3,4);

phi = atan2(T6(2,3),T6(1,3));

THy = T6(1,3)*cos(phi) + T6(2,3)*sin(phi);
THx = T6(3,3);
theta = atan2(THy,THx);

psiy = -T6(1,1)*sin(phi) + T6(2,1)*cos(phi);
psix = -T6(1,2)*sin(phi) + T6(2,2)*cos(phi);
psi = atan2(psiy,psix);

phi = rad2deg(phi);
theta = rad2deg(theta);
psi = rad2deg(psi);

out = T6;
