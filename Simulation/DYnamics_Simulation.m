close all; clc

g = 9.801; % gravity constant

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
l1 = 0.235; % length [m]
d1 = 0.16954384; % mass center
m1 = 0.2249; % mass [kg]
% moment of inertia
I1xx = 0.00096320; I1xy = -0.00000008; I1xz = -0.00000477;
I1yx = -0.00000008; I1yy = 0.00098847; I1yz = 0.00001586;
I1zx = -0.00000477; I1zy = 0.00001586; I1zz = 0.00006400;
%======================================  link 2
l2 = 0.279; % length [m]
d2 = 0.13069137; % mass center
m2 = 0.3142; % mass [kg]
% moment of inertia
I2xx = 0.00073785; I2xy = 0.00000000; I2xz = 0.00000000;
I2yx = 0.00000000; I2yy = 0.00083426; I2yz = 0.00000763;
I2zx = 0.00000000; I2zy = 0.00000763; I2zz = 0.00015999;

%% %%%%%%%%%%%%%%%%%%% discrete time
T = 5; % second
N = 620; % resolution
i = 0; 
for t = linspace(0, T, N)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% instantaneous time
    i = i + 1; time(i) = t;
    %%%%%%%%%%%%%%% Joint 1: angular displacement, velocity, acceleration
    theta__1(i) = 0; %A1*sin(f1*t); 
    theta__dot1(i) = 0; %A1*f1*cos(f1*t); 
    theta__ddot1(i) = 0; %-A1*f1^2*sin(f1*t); 
    
    %%%%%%%%%%%%%%% Joint 2: angular displacement, velocity, acceleration
    theta__2(i) = (3/50)*pi*t^2-(1/125)*pi*t^3; % A2*sin(f2*t); 
    theta__dot2(i) = (3/25)*pi*t-(3/125)*pi*t^2; %A2*f2*cos(f2*t); %plus 45degrees
    theta__ddot2(i) = (3/25)*pi-(6/125)*pi*t; %-A2*f2^2*sin(f2*t); %plus 45degrees
    
    %%%%%%%%%%%%%%% Joint 2: angular displacement, velocity, acceleration
    theta__3(i) = 0; %A3*sin(f3*t); 
    theta__dot3(i) = 0; %A3*f3*cos(f3*t); %plus 45degrees
    theta__ddot3(i) = 0; %-A3*f3^2*sin(f3*t); %plus 45degrees
%% mass Matrics
 H11 = (1/2)*(2*I1zz+2*I2zz+d2.^2*m2+l1.^2*m2+l1.^2*m2*cos(2*theta__2(i) ...
  )+2*d2*l1*m2*cos(theta__3(i))+d2.^2*m2*cos(2*(theta__2(i)+theta__3(i)))+2* ...
  d2*l1*m2*cos(2*theta__2(i)+theta__3(i)));
 H12 = (1/2)*((-1)*(I1yz+I1zy+I2yz+ ...
  I2zy)*cos(theta__1(i))+(I1xz+I1zx+I2xz+I2zx)*sin(theta__1(i)));
 H13 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta__1(i))+(I2xz+I2zx)*sin(theta__1(i)));

 H21 = (1/2)*((-1)*(I1yz+I1zy+I2yz+I2zy)*cos(theta__1(i)) ...
     +(I1xz+I1zx+I2xz+I2zx)*sin(theta__1(i)));
 H22 = (1/2)*(I1xx+I1yy+I2xx+I2yy+2*d2.^2*m2+2*l1.^2*m2+((-1)* ...
  I1xx+I1yy+(-1)*I2xx+I2yy)*cos(2*theta__1(i))+4*d2*l1*m2*cos(theta__3(i) ...
  )+(-1)*I1xy*sin(2*theta__1(i))+(-1)*I1yx*sin(2*theta__1(i))+(-1)*I2xy* ...
  sin(2*theta__1(i))+(-1)*I2yx*sin(2*theta__1(i)));
 H23 = (1/2)*(I2xx+I2yy+2* ...
  d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta__1(i))+2*d2*l1*m2*cos(theta__3(i)) ...
  +(-1)*I2xy*sin(2*theta__1(i))+(-1)*I2yx*sin(2*theta__1(i)));

 H31 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta__1(i))+(I2xz+I2zx)*sin(theta__1(i)));
 H32 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta__1(i))+2*d2* ...
  l1*m2*cos(theta__3(i))+(-1)*I2xy*sin(2*theta__1(i))+(-1)*I2yx*sin(2*theta__1(i)));
 H33 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2* ...
  theta__1(i))+(-1)*(I2xy+I2yx)*sin(2*theta__1(i)));

 C11 = (-2)*m2*(l1*cos(theta__2(i))+d2*cos(theta__2(i)+theta__3(i)))*((l1*sin(theta__2(i)) ...
  +d2*sin(theta__2(i)+theta__3(i)))*theta__dot2(i)+d2*sin(theta__2(i)+theta__3(i)) ...
  *theta__dot3(i));
 C12 = (1/2)*((I1xy+I1yx+I2xy+I2yx)*cos(2*theta__1(i) ...
  )+((-1)*I1xx+I1yy+(-1)*I2xx+I2yy)*sin(2*theta__1(i)))*theta__dot2(i);
 C13 = (1/2)*((I2xy+I2yx)*cos(2*theta__1(i))+((-1)*I2xx+I2yy)* ...
  sin(2*theta__1(i)))*(2*theta__dot2(i)+theta__dot3(i));

 C21 = (1/2)*((I1xz+I1zx+I2xz+I2zx)*cos(theta__1(i))+(I1yz+I1zy+I2yz+I2zy)* ...
  sin(theta__1(i))+m2*(l1.^2*sin(2*theta__2(i))+d2.^2*sin(2*(theta__2(i)+theta__3(i)))+ ...
  2*d2*l1*sin(2*theta__2(i)+theta__3(i))))*theta__dot1(i)+(-1)*(( ...
  I1xy+I1yx+I2xy+I2yx)*cos(2*theta__1(i))+((-1)*I1xx+I1yy+(-1)*I2xx+ ...
  I2yy)*sin(2*theta__1(i)))*theta__dot2(i)+(-1)*((I2xy+I2yx)* ...
  cos(2*theta__1(i))+((-1)*I2xx+I2yy)*sin(2*theta__1(i)))*theta__dot3(i);
 C22 = (-2)*d2*l1*m2*sin(theta__3(i))*theta__dot3(i);
 C23 = (-1)*d2*l1*m2*sin(theta__3(i))*theta__dot3(i);

 C31 = (1/2)*((I2xz+I2zx)*cos( ...
  theta__1(i))+(I2yz+I2zy)*sin(theta__1(i))+2*d2*m2*(l1*cos(theta__2(i))+d2*cos( ...
  theta__2(i)+theta__3(i)))*sin(theta__2(i)+theta__3(i)))*theta__dot1(i)+(-1)*(( ...
  I2xy+I2yx)*cos(2*theta__1(i))+((-1)*I2xx+I2yy)*sin(2*theta__1(i)))*( ...
  theta__dot2(i)+theta__dot3(i));
 C32 = d2*l1*m2*sin(theta__3(i))*theta__dot2(i);
 C33 = 0;
 
 F1 = 0.25*2*(1/(1 + exp(-2*theta__dot1(i))) - 0.5) + (0.005*(theta__dot1(i))); 
 F2 = 0.685*2*(1/(1 + exp(-2*theta__dot2(i))) - 0.5) + (0.02*(theta__dot2(i)));
 F3 = 0.39*2*(1/(1 + exp(-5*theta__dot3(i))) - 0.5) + (0.008*(theta__dot3(i)));

G1 = 0;
G2 = g*((d1*m1+l1*m2)*cos(theta__2(i))+d2*m2*cos(theta__2(i)+theta__3(i)));
G3 = d2*g*m2*cos(theta__2(i)+theta__3(i));        

%V = [C11 + C12 + C13; C21 + C22 + C23; C31 + C32 + C33]


%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link positions
    %%%%%%%%%%%%%%%%%% link 1
    X1(i) = l1*cos(theta__1(i))*cos(theta__2(i));
    Y1(i) = l1*sin(theta__1(i))*cos(theta__2(i));
    Z1(i) = l1*sin(theta__2(i));
   % x1(i) = 0.5*X1(i); y1(i) = 0.5*Y1(i); 
    %%%%%%%%%%%%%%%%%% link 2
    X2(i) = X1(i) + l2*cos(theta__2(i) + theta__3(i))*cos(theta__1(i)); 
    Y2(i) = Y1(i) + l2*cos(theta__2(i) + theta__3(i))*sin(theta__1(i));
    Z2(i) = Z1(i) + l2*sin(theta__2(i) + theta__3(i));
    %x2(i) = X1(i) + 0.5*X2(i); %Y2(i) = Y1(i) + 0.5*Y2(i);

%% DYnamics Matrics
M = [H11, H12 , H13; H21 , H22 , H23; H31 , H32 ,H33];
V = [C11*theta__dot1(i)+C12*theta__dot2(i)+C13*theta__dot3(i);C21*theta__dot1(i)+C22*theta__dot2(i)+C23*theta__dot3(i);C31*theta__dot1(i)+C32*theta__dot2(i)+C33*theta__dot3(i)];
G = [G1 ; G2 ; G3];
Theta__ddot = [theta__ddot1(i);theta__ddot2(i);theta__ddot3(i)];
F = [F1;F2;F3];

beta =[C11 C12 C13; C21 C22 C23; C31 C32 C33]+[G1;G2;G3];


%% compute Torque
tauVector = M*Theta__ddot+beta;
tau1(i) = tauVector(1);
tau2(i) = tauVector(2);
tau3(i) = tauVector(3);

Tau_test = [0 0 0;1 0 0;0 0 0];

%% compute Acceleration 
acceleration = M\(tauVector-beta);
acc1(i) = acceleration(1);
acc2(i) = acceleration(2);
acc3(i) = acceleration(3);

end
time1 = 0:0.008:619*0.008;
%VarName7(621) = [];
%VarName8(621) = [];
%VarName9(621) = [];
%%
figure(2)
clf
figure(2)
subplot(3, 1, 1)
hold on
plot(time, theta__1(i), 'b')
plot(time, theta__2, 'r')
plot(time, theta__3, 'g')
hold off
legend('theta 1', 'theta 2', 'theta 3')
grid on; 
xlabel('time [sec]'); ylabel('angular displacement [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, theta__dot1, 'b')
plot(time, theta__dot2, 'r')
plot(time, theta__dot3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular velocity [rad/s]'); 
subplot(3, 1, 3)
hold on
plot(time, theta__ddot1, 'b')
plot(time, theta__ddot2, 'r')
plot(time, theta__ddot3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular acceleration [rad/s^2]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motor torques
%figure(1)
%clf
%figure(1)
%hold on
%plot(time, tau_1, 'r')
%plot(time, tau_2, 'g')
%plot(time, tau_3, 'b')
%hold off
%legend(' joint 1', 'joint 2', 'joint 3')
%grid on; 
%xlabel('time [sec]'); ylabel('torques [Nm/rad]'); 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motor acc
figure(4)
clf
figure(4)
hold on
plot(time, acc1, 'r')
plot(time, acc2, 'g')
plot(time, acc3, 'b')
hold off
legend(' joint 1', 'joint 2', 'joint 3')
grid on; 
xlabel('time [sec]'); ylabel('acc [RAD/s^2]'); 


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motor tau
figure(5)
clf
figure(5)
hold on
title('Computed and Measured Torque - Joint 2')
%plot(time, tau1, 'r')
plot(time, tau2, '-.b','Linewidth',4)
%plot(time, tau3, 'b')
%plot(VarName2)
plot(time1,smooth(VarName8,100),'r','Linewidth',4)
hold off
legend('Computed Torque', 'Measured Torque')
grid on; 
xlabel('Time [s]'); ylabel('torque [Nm]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Motion Profile
figure(7)
clf
figure(7)
hold on
title('Motion Profile - Joint 2')
%plot(time, tau1, 'r')
plot(time, theta__2, '-.r','Linewidth',4)
plot(time1, smooth(VarName9,100), 'r','Linewidth',4)

xlabel('Time [s]'); ylabel('Angular displacement [rad]');
yyaxis right;
plot(time, theta__dot2, '-.g', 'Linewidth',4)
plot(time1, smooth(VarName7,100), 'g', 'Linewidth',4)
ylim([0 1.6]);
ylabel('Velocity [rad/s]'); 
hold off
legend('Computed Angular Displacement', 'Messured Angular Displacement','Computed Angular Velocity', 'Messured Angular Velocity')
grid on;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
%figure(3)
%clf
%figure(3)
%hold on 
%xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
%set(gca,'NextPlot','replaceChildren');
%for j = 1 : N
%    plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'Linewidth', 2);
%    axis equal; grid on; axis([-0.55 0.55 -0.55 0.55 -0.55 0.55]);
%    plot3(X1(j), Y1(j), Z1(j), 'bo', 'MarkerSize', 6, 'Linewidth', 2);
%    plot3(X2(j), Y2(j), Z2(j), 'ro', 'MarkerSize', 6, 'Linewidth', 2);
%    line([0 X1(j)], [0 Y1(j)], [0 Z1(j)], 'Color', 'b', 'Linewidth', 2); 
%    line([X1(j) X2(j)], [Y1(j) Y2(j)], [Z1(j) Z2(j)], 'Color', 'r', 'Linewidth', 2);  
%    F(j) = getframe; 
%end
%hold off
%movie(F);
