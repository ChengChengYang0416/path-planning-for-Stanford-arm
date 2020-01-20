%% motion planning in cartesian space

% trajectory : A -> A' -> (B) -> C' -> C
% defind the orientation and position of A, B, C
clc;
clear;
close all;
sampling_time = 0.002;
T = 0.5;
t_acc = 0.2;

A = [ 0  1  0  20;
      0  0 -1  10;
     -1  0  0 -10;
      0  0  0   1];
B = [-1  0  0  20;
      0 -1  0  -5;
      0  0  1  10;
      0  0  0   1];
C = [ 1  0  0 -10;
      0  0 -1  15;
      0  1  0  25;
      0  0  0   1];

nA = [A(1,1);A(2,1);A(3,1)];
oA = [A(1,2);A(2,2);A(3,2)];
aA = [A(1,3);A(2,3);A(3,3)];
pA = [A(1,4);A(2,4);A(3,4)];
nB = [B(1,1);B(2,1);B(3,1)];
oB = [B(1,2);B(2,2);B(3,2)];
aB = [B(1,3);B(2,3);B(3,3)];
pB = [B(1,4);B(2,4);B(3,4)];
nC = [C(1,1);C(2,1);C(3,1)];
oC = [C(1,2);C(2,2);C(3,2)];
aC = [C(1,3);C(2,3);C(3,3)];
pC = [C(1,4);C(2,4);C(3,4)];

%% straight line portion motion planning from A to A'(-0.5s~-0.2s)

% solution of x, y, z, theta, phi, psi (in 21~23 of handout Ch5)
% i.e. the position and angular difference between pA and pB
x = dot(nA, (pB-pA));
y = dot(oA, (pB-pA));
z = dot(aA, (pB-pA));
psi = atan2(dot(oA, aB),dot(nA, aB));
theta = atan2(sqrt((dot(nA, aB))^2+(dot(oA, aB))^2),dot(aA, aB));
Sphi = -sin(psi)*cos(psi)*(1-cos(theta))*(dot(nA, nB))+((cos(psi))^2*(1-cos(theta))+cos(theta))*(dot(oA, nB))-sin(psi)*sin(theta)*(dot(aA, nB));
Cphi = -sin(psi)*cos(psi)*(1-cos(theta))*(dot(nA, oB))+((cos(psi))^2*(1-cos(theta))+cos(theta))*(dot(oA, oB))-sin(psi)*sin(theta)*(dot(aA, oB));
phi = atan2(Sphi,Cphi);

% planing for linear portion
% A to A'(-0.5s~-0.2s)
dataA = 1;
for t = -0.5:sampling_time:-0.2
    h = (t+T)/T;
    
    % position and angle accumulation every time step
    dx = x*h;
    dy = y*h;
    dz = z*h;
    dpsi = psi*h;
    dtheta = theta*h;
    dphi = phi*h; 
    
    S_psi = sin(psi);
    C_psi = cos(psi);
    S_theta = sin(dtheta);
    C_theta = cos(dtheta);
    V_theta = 1-C_theta;
    S_phi = sin(dphi);
    C_phi = cos(dphi);
    
    % Dr = Tr*Rar*Ror
    % Dr accumulation every time step
    Tr = [1 0 0 dx;
          0 1 0 dy;
          0 0 1 dz;
          0 0 0 1];
      
    Rar = [   S_psi^2*V_theta+C_theta     -S_psi*C_psi*V_theta    C_psi*S_theta   0;
              -S_psi*C_psi*V_theta        C_psi^2*V_theta+C_theta S_psi*S_theta   0;
              -C_psi*S_theta              -S_psi*S_theta          C_theta         0;
              0                           0                       0               1];
   
    Ror = [   C_phi                -S_phi        0  0;
              S_phi                C_phi         0  0;
              0                    0             1  0;
              0                    0             0  1];
    Dr = Tr*Rar*Ror;
    
    % update A matrix every time step
    pA_line(:, :, dataA) = A*Dr;
    xA_line(:, dataA) = pA_line(1, 4, dataA);
    yA_line(:, dataA) = pA_line(2, 4, dataA);
    zA_line(:, dataA) = pA_line(3, 4, dataA);  
    dataA = dataA + 1;
end

%% transition portion motion planning from A' to C' (-0.2s~0.2s)

% A matrix of position A' (last potint of line portion)
A_prime = pA_line(:, :, dataA-1);          
nA_prime = [A_prime(1, 1); A_prime(2, 1); A_prime(3, 1)];
oA_prime = [A_prime(1, 2); A_prime(2, 2); A_prime(3, 2)];
aA_prime = [A_prime(1, 3); A_prime(2, 3); A_prime(3, 3)];
pA_prime = [A_prime(1, 4); A_prime(2, 4); A_prime(3, 4)];

% position and angular difference between pA' and pB (in 10 of handout Ch5)
xA = dot(nB, (pA_prime-pB));
yA = dot(oB, (pA_prime-pB));
zA = dot(aB, (pA_prime-pB));
psiA = atan2(dot(oB, aA_prime), dot(nB, aA_prime));
thetaA = atan2(sqrt((dot(nB, aA_prime))^2+(dot(oB, aA_prime))^2), dot(aB, aA_prime));
SphiA = -sin(psiA)*cos(psiA)*(1-cos(thetaA))*(dot(nB, nA_prime))+((cos(psiA))^2*(1-cos(thetaA))+cos(thetaA))*(dot(oB, nA_prime))-sin(psiA)*sin(thetaA)*(dot(aB, nA_prime));
CphiA = -sin(psiA)*cos(psiA)*(1-cos(thetaA))*(dot(nB, oA_prime))+((cos(psiA))^2*(1-cos(thetaA))+cos(thetaA))*(dot(oB, oA_prime))-sin(psiA)*sin(thetaA)*(dot(aB, oA_prime));
phiA = atan2(SphiA, CphiA);

% position and angular difference between pC and pB (in 10 of handout Ch5)
xC = dot(nB, (pC-pB));
yC = dot(oB, (pC-pB));
zC = dot(aB, (pC-pB));
psiC = atan2(dot(oB, aC), dot(nB, aC));
thetaC = atan2(sqrt((dot(nB, aC))^2+(dot(oB, aC))^2), dot(aB, aC));
SphiC = -sin(psiC)*cos(psiC)*(1-cos(thetaC))*(dot(nB, nC))+((cos(psiC))^2*(1-cos(thetaC))+cos(thetaC))*(dot(oB, nC))-sin(psiC)*sin(thetaC)*(dot(aB, nC));
CphiC = -sin(psiC)*cos(psiC)*(1-cos(thetaC))*(dot(nB, oC))+((cos(psiC))^2*(1-cos(thetaC))+cos(thetaC))*(dot(oB, oC))-sin(psiC)*sin(thetaC)*(dot(aB, oC));
phiC = atan2(SphiC, CphiC);

if abs(psiC-psiA) > pi/2
    psiA = psiA+pi;
    thetaA = -thetaA;
end

% planing for transition portion
% A' to C'(-0.2s~0.2s)
dataB = 1;
for t = -0.198:sampling_time:0.198
    h = (t+t_acc)/(2*t_acc);
    
    % position and angle accumulation every time step
    dx_B = ((xC*0.2/0.5+xA)*(2-h)*h^2-2*xA)*h+xA;
    dy_B = ((yC*0.2/0.5+yA)*(2-h)*h^2-2*yA)*h+yA;
    dz_B = ((zC*0.2/0.5+zA)*(2-h)*h^2-2*zA)*h+zA;
    dpsi_B = (psiC-psiA)*h+psiA;
    dtheta_B = ((thetaC*0.2/0.5+thetaA)*(2-h)*h^2-2*thetaA)*h+thetaA;
    dphi_B = ((phiC*0.2/0.5+phiA)*(2-h)*h^2-2*phiA)*h+phiA;
   
    S_psi_B = sin(dpsi_B);
    C_psi_B = cos(dpsi_B);
    S_theta_B = sin(dtheta_B);
    C_theta_B = cos(dtheta_B);
    V_theta_B = 1-C_theta_B;
    S_phi_B = sin(dphi_B);
    C_phi_B = cos(dphi_B);
    
    % Dr=Tr*Rar*Ror
    % Dr accumulation every time step
    Tr_B = [1 0 0 dx_B;
            0 1 0 dy_B;
            0 0 1 dz_B;
            0 0 0    1];
    Rar_B = [ S_psi_B^2*V_theta_B+C_theta_B    -S_psi_B*C_psi_B*V_theta_B        C_psi_B*S_theta_B   0;
              -S_psi_B*C_psi_B*V_theta_B        C_psi_B^2*V_theta_B+C_theta_B    S_psi_B*S_theta_B   0;
              -C_psi_B*S_theta_B               -S_psi_B*S_theta_B                C_theta_B           0;
              0                                          0                       0                   1];
    Ror_B = [ C_phi_B  -S_phi_B      0  0;
              S_phi_B   C_phi_B      0  0;
              0           0          1  0;
              0           0          0  1];
    Dr_B = Tr_B*Rar_B*Ror_B;   
                               
  
    pB_tran(:, :, dataB) = B*Dr_B;
    xB_tran(:, dataB) = pB_tran(1, 4, dataB);
    yB_tran(:, dataB) = pB_tran(2, 4, dataB);
    zB_tran(:, dataB) = pB_tran(3, 4, dataB);  
    dataB = dataB+1;
end


%% straight line portion motion planning from C' to C (0.2s~0.5s)

% planing for linear portion
dataC = 1;
for t = 0.2:sampling_time:0.5
    h = t/0.5;
    
    % position and angle accumulation every time step
    dx_C = xC*h;
    dy_C = yC*h;
    dz_C = zC*h;
    dpsi_C = psiC;
    dtheta_C = thetaC*h;
    dphi_C = phiC*h;
    
    S_psi_C = sin(dpsi_C);
    C_psi_C = cos(dpsi_C);
    S_theta_C = sin(dtheta_C);
    C_theta_C = cos(dtheta_C);
    V_theta_C = 1-C_theta_C;
    S_phi_C = sin(dphi_C);
    C_phi_C = cos(dphi_C);
    
    % Dr = Tr*Rar*Ror
    % Dr accumulation every time step
    Tr_C = [1 0 0 dx_C;
            0 1 0 dy_C;
            0 0 1 dz_C;
            0 0 0    1];
    Rar_C = [ S_psi_C^2*V_theta_C+C_theta_C    -S_psi_C*C_psi_C*V_theta_C        C_psi_C*S_theta_C   0;
              -S_psi_C*C_psi_C*V_theta_C        C_psi_C^2*V_theta_C+C_theta_C    S_psi_C*S_theta_C   0;
              -C_psi_C*S_theta_C               -S_psi_C*S_theta_C                C_theta_C           0;
                0                                          0                     0                   1];
    Ror_C = [ C_phi_C  -S_phi_C      0  0;
              S_phi_C   C_phi_C      0  0;
              0           0          1  0;
              0           0          0  1];
    Dr_C = Tr_C*Rar_C*Ror_C;   
    pC_line(:, :, dataC) = B*Dr_C;
    xC_line(:, dataC) = pC_line(1, 4, dataC);
    yC_line(:, dataC) = pC_line(2, 4, dataC);
    zC_line(:, dataC) = pC_line(3, 4, dataC);  
    dataC = dataC+1;
end

% combine the trajectory A -> A' -> (B) -> C' -> C
X = [xA_line xB_tran xC_line];
Y = [yA_line yB_tran yC_line];
Z = [zA_line zB_tran zC_line];

%% plot position, velocity, acceleration A -> A' -> (B) -> C' -> C
% position A -> A' -> (B) -> C' -> C
figure(1)
t = 0:sampling_time:1;
subplot(3, 1, 1);
plot(t, X);
title('position of x');

subplot(3,1,2);
plot(t, Y);
ylabel('Position(cm)')
title('position of y');

subplot(3, 1, 3);
plot(t, Z);
title('position of z');
xlabel('Time(s)')


% velocity A -> A' -> (B) -> C' -> C
figure(2)
dt=t(2:501);
dX = diff(X)/0.002;
dY = diff(Y)/0.002;
dZ = diff(Z)/0.002;

subplot(3, 1, 1);
plot(dt, dX);
title('velocity of x');

subplot(3, 1, 2);
plot(dt, dY);
title('velocity of y');
ylabel('Velocity(cm/s)');

subplot(3, 1, 3);
plot(dt, dZ);
title('velocity of z');
xlabel('Time(s)')

% acceleration A -> A' -> (B) -> C' -> C
figure(3)
dt2 = t(3:501);
dX2 = diff(dX)/0.002;
dY2 = diff(dY)/0.002;
dZ2 = diff(dZ)/0.002;
subplot(3, 1, 1);
plot(dt2, dX2);
title('acceleration of x');

subplot(3, 1, 2);
plot(dt2, dY2);
title('acceleration of y');
ylabel('Acceleration(cm/s^2)');

subplot(3, 1, 3);
plot(dt2, dZ2);
title('acceleration of z');
xlabel('Time(s)')

%% 3D plot A -> A' -> (B) -> C' -> C
% trajectory A -> A' -> (B) -> C' -> C
figure(4)
set(gcf,'Units','centimeters','position',[5 5 32 18]);
plot3(xA_line, yA_line, zA_line, 'r');
hold on;
plot3(xB_tran, yB_tran, zB_tran, 'g');
hold on;
plot3(xC_line, yC_line, zC_line, 'b');
hold on;  

% orientation of A, B, C
hold on;
plot3([pA(1), pA(1)+nA(1)*3], [pA(2), pA(2)+nA(2)*3], [pA(3), pA(3)+nA(3)*3],'r');
hold on;
plot3([pA(1), pA(1)+oA(1)*3], [pA(2), pA(2)+oA(2)*3], [pA(3), pA(3)+oA(3)*3],'g');
hold on;
plot3([pA(1), pA(1)+aA(1)*3], [pA(2), pA(2)+aA(2)*3], [pA(3), pA(3)+aA(3)*3],'b');
hold on;
plot3([pB(1), pB(1)+nB(1)*3], [pB(2), pB(2)+nB(2)*3], [pB(3), pB(3)+nB(3)*3],'r');
hold on;
plot3([pB(1), pB(1)+oB(1)*3], [pB(2), pB(2)+oB(2)*3], [pB(3), pB(3)+oB(3)*3],'g');
hold on;
plot3([pB(1), pB(1)+aB(1)*3], [pB(2), pB(2)+aB(2)*3], [pB(3), pB(3)+aB(3)*3],'b');
hold on;
plot3([pC(1), pC(1)+nC(1)*3], [pC(2), pC(2)+nC(2)*3], [pC(3), pC(3)+nC(3)*3],'r');
hold on;
plot3([pC(1), pC(1)+oC(1)*3], [pC(2), pC(2)+oC(2)*3], [pC(3), pC(3)+oC(3)*3],'g');
hold on;
plot3([pC(1), pC(1)+aC(1)*3], [pC(2), pC(2)+aC(2)*3], [pC(3), pC(3)+aC(3)*3],'b');
hold on;

% dotted line A -> B -> C
plot3([pA(1), pB(1)], [pA(2), pB(2)], [pA(3), pB(3)],'LineStyle', ':', 'color', 'k');
hold on;
plot3([pB(1), pC(1)], [pB(2), pC(2)], [pB(3), pC(3)],'LineStyle', ':', 'color', 'k');

xlabel('X-axis(cm)');
ylabel('Y-axis(cm)');
zlabel('Z-axis(cm)');
text(20,10,-15,'A(20,10,-10)');
text(xA_line(151),yA_line(151),zA_line(151)-3, sprintf('A・(%.0f,%.0f,%.0f)',xA_line(151),yA_line(151),zA_line(151)));
text(22,-5,10,'B(20,-5,10)');
text(xC_line(1),yC_line(1),zC_line(1)+3, sprintf('C・(%.0f,%.0f,%.0f)', xC_line(1),yC_line(1),zC_line(1)));
text(-10,17,31.5,'C(-10,15,25)');
xlim([-15 25]);
ylim([-10 20]);
zlim([-15 30]);
title('3D path of Cartesion Motion')
grid

%% 3D oriention plot A -> A' -> (B) -> C' -> C
% trajectory A -> A' -> (B) -> C' -> C
figure(5)
set(gcf,'Units','centimeters','position',[5 5 32 18]);
plot3(xA_line, yA_line, zA_line, 'r');
hold on;
plot3(xB_tran, yB_tran, zB_tran, 'g');
hold on;
plot3(xC_line, yC_line, zC_line, 'b');
hold on;  

% orientation of A, B, C
plot3([pA(1), pA(1)+nA(1)*3], [pA(2), pA(2)+nA(2)*3], [pA(3), pA(3)+nA(3)*3],'r');
hold on;
plot3([pA(1), pA(1)+oA(1)*3], [pA(2), pA(2)+oA(2)*3], [pA(3), pA(3)+oA(3)*3],'g');
hold on;
plot3([pA(1), pA(1)+aA(1)*3], [pA(2), pA(2)+aA(2)*3], [pA(3), pA(3)+aA(3)*3],'b');
hold on;
plot3([pB(1), pB(1)+nB(1)*3], [pB(2), pB(2)+nB(2)*3], [pB(3), pB(3)+nB(3)*3],'r');
hold on;
plot3([pB(1), pB(1)+oB(1)*3], [pB(2), pB(2)+oB(2)*3], [pB(3), pB(3)+oB(3)*3],'g');
hold on;
plot3([pB(1), pB(1)+aB(1)*3], [pB(2), pB(2)+aB(2)*3], [pB(3), pB(3)+aB(3)*3],'b');
hold on;
plot3([pC(1), pC(1)+nC(1)*3], [pC(2), pC(2)+nC(2)*3], [pC(3), pC(3)+nC(3)*3],'r');
hold on;
plot3([pC(1), pC(1)+oC(1)*3], [pC(2), pC(2)+oC(2)*3], [pC(3), pC(3)+oC(3)*3],'g');
hold on;
plot3([pC(1), pC(1)+aC(1)*3], [pC(2), pC(2)+aC(2)*3], [pC(3), pC(3)+aC(3)*3],'b');
hold on;

% dotted line A -> B -> C
plot3([pA(1), pB(1)], [pA(2), pB(2)], [pA(3), pB(3)],'LineStyle', ':', 'color', 'k');
hold on;
plot3([pB(1), pC(1)], [pB(2), pC(2)], [pB(3), pC(3)],'LineStyle', ':', 'color', 'k');

% orientation of every time step
s=1;  
for j1=-0.5:sampling_time:-0.2
    hold on;
    plot3([xA_line(s),xA_line(s)+pA_line(1,3,s)*2],[yA_line(s),yA_line(s)+pA_line(2,3,s)*2],[zA_line(s),zA_line(s)+pA_line(3,3,s)*2], 'r');
    s=s+1;
end
s=1;                                          
for j2=-0.198:sampling_time:0.198
    hold on;
    plot3([xB_tran(s),xB_tran(s)+pB_tran(1,3,s)*2],[yB_tran(s),yB_tran(s)+pB_tran(2,3,s)*2],[zB_tran(s),zB_tran(s)+pB_tran(3,3,s)*2], 'g');
    s=s+1;
end
s=1;                                          
for j3=0.2:sampling_time:0.5
    hold on;
    plot3([xC_line(s),xC_line(s)+pC_line(1,3,s)*2],[yC_line(s),yC_line(s)+pC_line(2,3,s)*2],[zC_line(s),zC_line(s)+pC_line(3,3,s)*2], 'b');
    s=s+1;
end

xlabel('X-axis(cm)');
ylabel('Y-axis(cm)');
zlabel('Z-axis(cm)');
text(20,10,-15,'A(20,10,-10)');
text(xA_line(151),yA_line(151),zA_line(151)-3, sprintf('A・(%.0f,%.0f,%.0f)',xA_line(151),yA_line(151),zA_line(151)));
text(22,-5,10,'B(20,-5,10)');
text(xC_line(1),yC_line(1),zC_line(1)+3, sprintf('C・(%.0f,%.0f,%.0f)', xC_line(1),yC_line(1),zC_line(1)));
text(-10,17,31.5,'C(-10,15,25)');
xlim([-15 25]);
ylim([-10 20]);
zlim([-15 30]);
title('3D path of Cartesion Motion')
grid
