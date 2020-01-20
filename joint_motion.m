%% motion planning in joint space

% trajectory : A -> A' -> (B) -> C' -> C
% defind the orientation and position of A, B, C
clc;
clear;
close all;
sampling_time = 0.002;
T = 0.5;
t_acc = 0.2;

A = [ 0  1  0  20
      0  0 -1  10
     -1  0  0 -10
      0  0  0   1];
B = [-1  0  0  20
      0 -1  0  -5
      0  0  1  10
      0  0  0   1];
C = [ 1  0  0 -10
      0  0 -1  15
      0  1  0  25
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

% Inverse Kinematics of A, B, C
thetaA = inverse_kinematics(A);                      
thetaB = inverse_kinematics(B);                      
thetaC = inverse_kinematics(C);   
thetaA = thetaA(3, :);
thetaB = thetaB(3, :);
thetaC = thetaC(4, :);

%%  line portion motion planning from A to A' (-0.5~-0.2)

thetaA_prime = thetaA+(thetaB-thetaA)/0.5*(0.5-0.2);      % joint variable of A'
dB = thetaA-thetaB;
dC = thetaC-thetaB;

dataA = 1;
for t = -0.5:sampling_time:-0.2 
    dthetaA(:,dataA) = thetaB-dB/0.5*t;
    domegaA(:,dataA) = -dB/0.5;
    dalphaA(:,dataA) = [0;0;0;0;0;0];
    
    % Forward Kinematics from Joint Path to Cartestion Path
    p1(:,:,dataA)=forward_kinematics(dthetaA(:,dataA)');
    x1(dataA)=p1(1,4,dataA);
    y1(dataA)=p1(2,4,dataA);
    z1(dataA)=p1(3,4,dataA);
    
    dataA = dataA+1;
end

%% transition portion motion planning from A' to C' (-0.2~0.2)

dataB = 1;
for t = -0.198:sampling_time:0.198
    h = (t+0.2)/0.4;
    dthetaB(:,dataB) = (0.2/0.5)*((dC+dB)*(2-h)*(h.^2)-2*dB)*h+dB*(0.2/0.5)+thetaB;
    domegaB(:,dataB) = ((dC+dB)*(1.5-h)*2*(h.^2)-dB)/0.5;
    dalphaB(:,dataB) = (dC+dB)*(1-h)*3*h/(0.2*0.5);
    
    % Forward Kinematics from Joint Path to Cartestion Path
    p2(:,:,dataB)=forward_kinematics(dthetaB(:,dataB)');
    x2(dataB)=p2(1,4,dataB);
    y2(dataB)=p2(2,4,dataB);
    z2(dataB)=p2(3,4,dataB);
    
    dataB = dataB+1;
end

%% line portion motion planning from C' to C (0.2~0.5)

dataC = 1;
for t = 0.2:sampling_time:0.5
    h = t/0.5;
    dthetaC(:,dataC) = dC*h+thetaB;
    domegaC(:,dataC) = dC/0.5;
    dalphaC(:,dataC) = [0;0;0;0;0;0];
    
    % Forward Kinematics from Joint Path to Cartestion Path
    p3(:,:,dataC)=forward_kinematics(dthetaC(:,dataC)');
    x3(dataC)=p3(1,4,dataC);
    y3(dataC)=p3(2,4,dataC);
    z3(dataC)=p3(3,4,dataC);
   
    dataC = dataC+1;
end

%% plot the angle, angular velocity, angular acceleration A -> A' -> (B) -> C' -> C
% plot the joint angle from joint1 to joint6
figure(1)
t = 0:sampling_time:1;
theta1=[dthetaA(1,:) dthetaB(1,:) dthetaC(1,:)];                     
subplot(3,2,1);
plot(t,theta1);
title('joint1');

theta2=[dthetaA(2,:) dthetaB(2,:) dthetaC(2,:)];                     
subplot(3,2,2);
plot(t,theta2);
title('joint2');

theta3=[dthetaA(3,:) dthetaB(3,:) dthetaC(3,:)];                     
subplot(3,2,3);
plot(t,theta3);
title('joint3');
ylabel('Angel(\circ) or Length(cm)');

theta4=[dthetaA(4,:) dthetaB(4,:) dthetaC(4,:)];                     
subplot(3,2,4);
plot(t,theta4);
title('joint4');

theta5=[dthetaA(5,:) dthetaB(5,:) dthetaC(5,:)];                     
subplot(3,2,5);
plot(t,theta5);
title('joint5');

theta6=[dthetaA(6,:) dthetaB(6,:) dthetaC(6,:)];                     
subplot(3,2,6);
plot(t,theta6);
title('joint6');

% plot the joint angular velocity from joint1 to joint6
figure(2);
subplot(3,2,1)
plot(t,[domegaA(1,:) domegaB(1,:) domegaC(1,:)]);                 
title('joint1');

subplot(3,2,2)
plot(t,[domegaA(2,:) domegaB(2,:) domegaC(2,:)]);                 
title('joint2');

subplot(3,2,3)
plot(t,[domegaA(3,:) domegaB(3,:) domegaC(3,:)]);                 
title('joint3');
ylabel('Angular Velocity(\circ/s) or Velocity(cm/s)');

subplot(3,2,4)
plot(t,[domegaA(4,:) domegaB(4,:) domegaC(4,:)]);                 
title('joint4');

subplot(3,2,5)
plot(t,[domegaA(5,:) domegaB(5,:) domegaC(5,:)]);                 
title('joint5');

subplot(3,2,6)
plot(t,[domegaA(6,:) domegaB(6,:) domegaC(6,:)]);                 
title('joint6');

% plot the joint angular acceleration from joint1 to joint6
figure(3)
subplot(3,2,1)
plot(t,[dalphaA(1,:) dalphaB(1,:) dalphaC(1,:)]);
title('joint1');

subplot(3,2,2)
plot(t,[dalphaA(2,:) dalphaB(2,:) dalphaC(2,:)]);
title('joint2');

subplot(3,2,3)
plot(t,[dalphaA(3,:) dalphaB(3,:) dalphaC(3,:)]);
title('joint3');
ylabel('Angular Acceleration(\circ/s^2) or Acceleration(cm/s^2)');

subplot(3,2,4)
plot(t,[dalphaA(4,:) dalphaB(4,:) dalphaC(4,:)]);
title('joint4');

subplot(3,2,5)
plot(t,[dalphaA(5,:) dalphaB(5,:) dalphaC(5,:)]);
title('joint5');

subplot(3,2,6)
plot(t,[dalphaA(6,:) dalphaB(6,:) dalphaC(6,:)]);
title('joint6');

%% 3D plot in Cartesian space
figure(4)
set(gcf,'Units','centimeters','position',[5 5 32 18]);
plot3(x1, y1, z1, 'r');
hold on;
plot3(x2, y2, z2, 'g');
hold on;
plot3(x3, y3, z3, 'b');
hold on;
plot3([pA(1),pA(1)+nA(1)*3],[pA(2),pA(2)+nA(2)*3],[pA(3),pA(3)+nA(3)*3],'r');
hold on;
plot3([pA(1),pA(1)+oA(1)*3],[pA(2),pA(2)+oA(2)*3],[pA(3),pA(3)+oA(3)*3],'g');
hold on;
plot3([pA(1),pA(1)+aA(1)*3],[pA(2),pA(2)+aA(2)*3],[pA(3),pA(3)+aA(3)*3],'b');
hold on;
plot3([pB(1),pB(1)+nB(1)*3],[pB(2),pB(2)+nB(2)*3],[pB(3),pB(3)+nB(3)*3],'r');
hold on;
plot3([pB(1),pB(1)+oB(1)*3],[pB(2),pB(2)+oB(2)*3],[pB(3),pB(3)+oB(3)*3],'g');
hold on;
plot3([pB(1),pB(1)+aB(1)*3],[pB(2),pB(2)+aB(2)*3],[pB(3),pB(3)+aB(3)*3],'b');
hold on;
plot3([pC(1),pC(1)+nC(1)*3],[pC(2),pC(2)+nC(2)*3],[pC(3),pC(3)+nC(3)*3],'r');
hold on;
plot3([pC(1),pC(1)+oC(1)*3],[pC(2),pC(2)+oC(2)*3],[pC(3),pC(3)+oC(3)*3],'g');
hold on;
plot3([pC(1),pC(1)+aC(1)*3],[pC(2),pC(2)+aC(2)*3],[pC(3),pC(3)+aC(3)*3],'b');
hold on;
plot3([pA(1), pB(1)], [pA(2), pB(2)], [pA(3), pB(3)],'LineStyle', ':', 'color', 'k');
hold on;
plot3([pB(1), pC(1)], [pB(2), pC(2)], [pB(3), pC(3)],'LineStyle', ':', 'color', 'k');

xlabel('X-axis(cm)');
ylabel('Y-axis(cm)');
zlabel('Z-axis(cm)');
text(20,10,-15,'A(20,10,-10)');
text(x1(151), y1(151), z1(151)-3, sprintf('A・(%.0f,%.0f,%.0f)',x1(151),y1(151),z1(151)));
text(22,-5,10,'B(20,-5,10)');
text(x3(1), y3(1), z3(1)+3, sprintf('C・(%.0f,%.0f,%.0f)', x3(1),y3(1),z3(1)));
text(-10,17,31.5,'C(-10,15,25)');
xlim([-15 25]);
ylim([-10 20]);
zlim([-15 30]);
title('3D path of Joint Motion')
grid

%% plot the oriention
figure(5)
set(gcf,'Units','centimeters','position',[5 5 32 18]);
plot3(x1,y1,z1,x2,y2,z2,x3,y3,z3);
hold on;
plot3([pA(1),pA(1)+nA(1)*3],[pA(2),pA(2)+nA(2)*3],[pA(3),pA(3)+nA(3)*3],'r');
hold on;
plot3([pA(1),pA(1)+oA(1)*3],[pA(2),pA(2)+oA(2)*3],[pA(3),pA(3)+oA(3)*3],'g');
hold on;
plot3([pA(1),pA(1)+aA(1)*3],[pA(2),pA(2)+aA(2)*3],[pA(3),pA(3)+aA(3)*3],'b');
hold on;
plot3([pB(1),pB(1)+nB(1)*3],[pB(2),pB(2)+nB(2)*3],[pB(3),pB(3)+nB(3)*3],'r');
hold on;
plot3([pB(1),pB(1)+oB(1)*3],[pB(2),pB(2)+oB(2)*3],[pB(3),pB(3)+oB(3)*3],'g');
hold on;
plot3([pB(1),pB(1)+aB(1)*3],[pB(2),pB(2)+aB(2)*3],[pB(3),pB(3)+aB(3)*3],'b');
hold on;
plot3([pC(1),pC(1)+nC(1)*3],[pC(2),pC(2)+nC(2)*3],[pC(3),pC(3)+nC(3)*3],'r');
hold on;
plot3([pC(1),pC(1)+oC(1)*3],[pC(2),pC(2)+oC(2)*3],[pC(3),pC(3)+oC(3)*3],'g');
hold on;
plot3([pC(1),pC(1)+aC(1)*3],[pC(2),pC(2)+aC(2)*3],[pC(3),pC(3)+aC(3)*3],'b');
hold on;
plot3([pA(1), pB(1)], [pA(2), pB(2)], [pA(3), pB(3)],'LineStyle', ':', 'color', 'k');
hold on;
plot3([pB(1), pC(1)], [pB(2), pC(2)], [pB(3), pC(3)],'LineStyle', ':', 'color', 'k');

s=1;  
for j1=-0.5:sampling_time:-0.2
    hold on;
    plot3([x1(s),x1(s)+p1(1,3,s)],[y1(s),y1(s)+p1(2,3,s)],[z1(s),z1(s)+p1(3,3,s)], 'r');
    s=s+1;
end
s=1;                                          
for j2=-0.198:sampling_time:0.198
    hold on;
    plot3([x2(s),x2(s)+p2(1,3,s)],[y2(s),y2(s)+p2(2,3,s)],[z2(s),z2(s)+p2(3,3,s)], 'g');
    s=s+1;
end
s=1;                                          
for j3=0.2:sampling_time:0.5
    hold on;
    plot3([x3(s),x3(s)+p3(1,3,s)],[y3(s),y3(s)+p3(2,3,s)],[z3(s),z3(s)+p3(3,3,s)], 'b');
    s=s+1;
end


xlabel('X-axis(cm)');
ylabel('Y-axis(cm)');
zlabel('Z-axis(cm)');
text(20,10,-15,'A(20,10,-10)');
text(x1(151), y1(151), z1(151)-3, sprintf('A・(%.0f,%.0f,%.0f)',x1(151),y1(151),z1(151)));
text(22,-5,10,'B(20,-5,10)');
text(x3(1), y3(1), z3(1)+3, sprintf('C・(%.0f,%.0f,%.0f)', x3(1),y3(1),z3(1)));
text(-10,17,31.5,'C(-10,15,25)');
xlim([-15 25]);
ylim([-10 20]);
zlim([-15 30]);
title('3D path of Joint Motion')
grid