clear all
close all
robot = Robot();
% Set varibles up
syms theta_1 theta_2 theta_3 l_1 l_2 l_3

% Solve for M, w, q, v, and the screw based on frame {0}
M0e = [1 0 0 (l_2+l_3)
     0 1 0 0
     0 0 1 l_1
     0 0 0 1];

M01 = [1 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 1];

M02 = [1 0 0 0
     0 1 0 0
     0 0 1 l_1
     0 0 0 1];

M03 = [1 0 0 l_2
     0 1 0 0
     0 0 1 l_1
     0 0 0 1];

wi0 = [0,0,0
      0,-1,-1
      1,0,0];

qi0 = [0,0,l_2
      0,0,0
      0,l_1,l_1];

vi0(:,1) = cross(-wi0(:,1),qi0(:,1));
vi0(:,2) = cross(-wi0(:,2),qi0(:,2));
vi0(:,3) = cross(-wi0(:,3),qi0(:,3));

Screwi0 = [wi0;vi0];

% Put exact values for lengths and thetas
AngArray = [theta_1; theta_2; theta_3];
thetalist0e = [theta_1,theta_2,theta_3];
thetalist03 = [theta_1,theta_2];
thetalist02 = [theta_1];

% Sub real values in
Slist = subs(Screwi0);
M0e = subs(M0e);
M03 = subs(M03);
M02 = subs(M02);
M01 = subs(M01);

% Solve for Transformation Matrix
T0e = simplify(FKinSpace(M0e,Slist,thetalist0e));
T03 = simplify(FKinSpace(M03,Slist(:,1:2),thetalist03));
T02 = simplify(FKinSpace(M02,Slist(:,1),thetalist02));
T01 = simplify(M01);

% Used T0e for the FK function for our robot.
% we calibrated the lab robot to make it match the one in HW4.


% Create Jacobian
z01 = T01(1:3,3);
z02 = T02(1:3,2);
z03 = T03(1:3,2);
z0e = T0e(1:3,3);
p01 = T01(1:3,4);
p02 = T02(1:3,4);
p03 = T03(1:3,4);
p0e = T0e(1:3,4);

J2 = [cross(z01,(p02-p01));
     z01];
J3 = [cross(z01,(p03-p01)), cross(z02,(p03-p02));
     z01, z02];
Je = [cross(z01,(p0e-p01)), cross(z02,(p0e-p02)), cross(z03,(p0e-p03));
     z01, z02, z03];

% Create unknown variables
syms t theta_1(t) theta_2(t) theta_3(t) m1 m2 m3 g
theta_1t = theta_1(t);
theta_2t = theta_2(t);
theta_3t = theta_3(t);
theta_1dot = diff(theta_1t,t);
theta_2dot = diff(theta_2t,t);
theta_3dot = diff(theta_3t,t);
AngArraydot = [theta_1dot; theta_2dot; theta_3dot];
theta_1ddot = diff(theta_1t,t,2);
theta_2ddot = diff(theta_2t,t,2);
theta_3ddot = diff(theta_3t,t,2);
AngArrayddot = [theta_1ddot; theta_2ddot; theta_3ddot];

% Setup I matrices
syms I_xx1 I_xy1 I_xz1 I_yx1 I_yy1 I_yz1 I_zx1 I_zy1 I_zz1
syms I_xx2 I_xy2 I_xz2 I_yx2 I_yy2 I_yz2 I_zx2 I_zy2 I_zz2
syms I_xx3 I_xy3 I_xz3 I_yx3 I_yy3 I_yz3 I_zx3 I_zy3 I_zz3
I1 = [I_xx1 I_xy1 I_xz1
      I_yx1 I_yy1 I_yz1
      I_zx1 I_zy1 I_zz1];
I2 = [I_xx2 I_xy2 I_xz2
      I_yx2 I_yy2 I_yz2
      I_zx2 I_zy2 I_zz2];
I3 = [I_xx3 I_xy3 I_xz3
      I_yx3 I_yy3 I_yz3
      I_zx3 I_zy3 I_zz3];

% solve for Jacobians using l_1c l_2c and l_3c terms
syms m1 m2 m3 theta_1 theta_2 theta_3 l_1 l_2 l_3 l_1c l_2c l_3c
l_1 = l_1c;
l_2 = l_2c;
l_3 = l_3c;

J2D = subs([J2,zeros(6,2)]);

syms l_1
J3D = subs([J3,zeros(6,1)]);

syms l_2
JeD = subs(Je);

% Solve for each componet of the D marix and sum them together
D1 = m1*transpose(J2D(1:3,:))*J2D(1:3,:) + transpose(J2D(4:6,:))*T02(1:3,1:3)*I1*transpose(T02(1:3,1:3))*J2D(4:6,:);
D2 = m2*transpose(J3D(1:3,:))*J3D(1:3,:) + transpose(J3D(4:6,:))*T03(1:3,1:3)*I2*transpose(T03(1:3,1:3))*J3D(4:6,:);
D3 = m3*transpose(JeD(1:3,:))*JeD(1:3,:) + transpose(JeD(4:6,:))*T0e(1:3,1:3)*I3*transpose(T0e(1:3,1:3))*JeD(4:6,:);

D = simplify(D1 + D2 + D3);

% Solve for C
for i = 1:3
    for j =1:3
        for k = 1:3
           Cijk(i,j,k) =  .5*(diff(D(k,j),AngArray(i)) + diff(D(k,i),AngArray(j)) - diff(D(i,j),AngArray(k)));
        end
    end
end

for j =1:3
    for k = 1:3
       C(k,j) = Cijk(1,j,k)*AngArraydot(1) + Cijk(2,j,k)*AngArraydot(2) + Cijk(3,j,k)*AngArraydot(3);
    end
end
Cnew = simplify(C);

% Solve for gravity term g(q)
l_1 = l_1c;
l_2 = l_2c;
l_3 = l_3c;

P1 = m1*-g*subs(T02(3,4));

syms l_1
P2 = m2*-g*subs(T03(3,4));

syms l_2
P3 = m3*-g*subs(T0e(3,4));

P = P1+P2+P3;

for ind = 1:3
    grav(ind,1) = diff(P,AngArray(ind));
end

gravnew = simplify(grav);

% Solve for other transfer functions and R's
T12 = simplify(inv(T01)*T02);
T23 = simplify(inv(T02)*T03);
T3e = simplify(inv(T03)*T0e);
R01 = T01(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);
R0e = T0e(1:3,1:3);
R12 = T12(1:3,1:3);
R23 = T23(1:3,1:3);
R3e = T3e(1:3,1:3);


% Find radius values to mass locations
syms l_1 l_2 l_3 l_1c l_2c l_3c
r1c1 = [0; 0; l_1c];
r2c1 = [0; 0; (l_1c-l_1)];
r12 = [0; 0; l_1];

r2c2 = [l_2c; 0; 0];
r3c2 = [(l_2c-l_2); 0; 0];
r23 = [l_2; 0; 0];

r3c3 = [l_3c; 0; 0];
rec3 = [(l_3c-l_3); 0; 0];
r3e = [l_3; 0; 0];

% Create array of angular and linear velocitys and accelerations
% Set base frame values to zero
angvelo = sym(zeros(3,4));
angaccel = sym(zeros(3,4));
accelC = sym(zeros(3,4));
accelE = sym(zeros(3,4));

% Forward Recursion
b = simplify([zeros(3,1), transpose(R01)*z01, transpose(R02)*z02, transpose(R03)*z03]);
angvelo(:,2) = transpose(R01)*angvelo(:,1) + b(:,2)*AngArraydot(1);
angvelo(:,3) = transpose(R12)*angvelo(:,2) + b(:,3)*AngArraydot(2);
angvelo(:,4) = transpose(R23)*angvelo(:,3) + b(:,4)*AngArraydot(3);

angaccel(:,2) = transpose(R01)*angaccel(:,1) + b(:,2)*AngArrayddot(1) + cross(angvelo(:,2),(b(:,2)*AngArraydot(1)));
angaccel(:,3) = transpose(R12)*angaccel(:,2) + b(:,3)*AngArrayddot(2) + cross(angvelo(:,3),(b(:,3)*AngArraydot(2)));
angaccel(:,4) = transpose(R23)*angaccel(:,3) + b(:,4)*AngArrayddot(3) + cross(angvelo(:,4),(b(:,4)*AngArraydot(3)));

accelE(:,2) = transpose(R01)*accelE(:,1) + cross(angaccel(:,2),r12) + cross(angvelo(:,2), cross(angvelo(:,2),r12));
accelE(:,3) = transpose(R12)*accelE(:,2) + cross(angaccel(:,3),r23) + cross(angvelo(:,3), cross(angvelo(:,3),r23));
accelE(:,4) = transpose(R23)*accelE(:,3) + cross(angaccel(:,4),r3e) + cross(angvelo(:,4), cross(angvelo(:,4),r3e));

accelC(:,2) = transpose(R01)*accelE(:,1) + cross(angaccel(:,2),r1c1) + cross(angvelo(:,2), cross(angvelo(:,2),r1c1));
accelC(:,3) = transpose(R12)*accelE(:,2) + cross(angaccel(:,3),r2c2) + cross(angvelo(:,3), cross(angvelo(:,3),r2c2));
accelC(:,4) = transpose(R23)*accelE(:,3) + cross(angaccel(:,4),r3c3) + cross(angvelo(:,4), cross(angvelo(:,4),r3c3));

% Backward Recursion to solve for force and torque
% Set Endeffector force and torque to zero

force(:,3) = R3e*[0;0;0] + m3*accelC(:,4) - m3*-transpose(R0e)*[0;0;g];
force(:,2) = R23*force(:,3) + m2*accelC(:,3) - m2*-transpose(R03)*[0;0;g];
force(:,1) = R12*force(:,2) + m1*accelC(:,2) - m1*-transpose(R02)*[0;0;g];

alltorque(:,3) = R3e* [0;0;0] - cross(force(:,3),r3c3) + cross((R3e*[0;0;0]),rec3) + I3*angaccel(:,4) + cross(angvelo(:,4),(I3*angvelo(:,4)));
alltorque(:,2) = R23*alltorque(:,3) - cross(force(:,2),r2c2) + cross((R23*force(:,3)),r3c2) + I2*angaccel(:,3) + cross(angvelo(:,3),(I2*angvelo(:,3)));
alltorque(:,1) = R12*alltorque(:,2) - cross(force(:,1),r1c1) + cross((R12*force(:,2)),r2c1) + I1*angaccel(:,2) + cross(angvelo(:,2),(I1*angvelo(:,2)));

% Pull out torque componets that affect the joint motor axis
torque = simplify([alltorque(3,1); alltorque(2,2); alltorque(2,3)]);

% Seperate terms
[tao1a,tao1b] = coeffs(torque(1), [theta_1ddot,theta_2ddot,theta_3ddot,g]);
[tao1aC,tao1bC] = coeffs(tao1a(4), [theta_1dot,theta_2dot,theta_3dot]);
[tao2a,tao2b] = coeffs(torque(2), [theta_1ddot,theta_2ddot,theta_3ddot,g]);
[tao2aC,tao2bC] = coeffs(tao2a(5), [theta_1dot,theta_2dot,theta_3dot]);
[tao3a,tao3b] = coeffs(torque(3), [theta_1ddot,theta_2ddot,theta_3ddot,g]);
[tao3aC,tao3bC] = coeffs(tao3a(5), [theta_1dot,theta_2dot,theta_3dot]);

Dnewton = simplify([tao1a(1), tao1a(2), tao1a(3)
                    tao2a(1), tao2a(2), tao2a(3)
                    tao3a(1), tao3a(2), tao3a(3)]);
gravitynewton = simplify([0; tao2a(4)*g; tao3a(4)*g]);
Cnewton = simplify([tao1aC(1)*AngArraydot(2) + tao1aC(2)*AngArraydot(3), tao1aC(3)*AngArraydot(2) + tao1aC(4)*AngArraydot(3), tao1aC(5)*AngArraydot(3);
                    tao2aC(1)*AngArraydot(1) + tao2aC(2)*AngArraydot(2), tao2aC(4)*AngArraydot(3), tao2aC(3)*AngArraydot(1) + tao2aC(5)*AngArraydot(3);
                    tao3aC(1)*AngArraydot(1) + tao3aC(2)*AngArraydot(2), tao3aC(4)*AngArraydot(2), tao3aC(3)*AngArraydot(1)]);

%[l_1, l_2, l_3, l_1c, l_2c, l_3c] = deal(0.3);
%THIS IS FOR OUR ROBOT
l_1 = .096326 ;%in meters
l_2 = .130231;
l_3 = .124;
l_1c = l_1;
l_2c = l_2;
l_3c = l_3;

%[m1, m2, m3] = deal(0.5);
m1 = .082;
m2= .082;
m3 = .170;
g = 9.8;

% robot.moveToSamePos(1, [0,-40,55,0], 1000);
jointsAngles(1,:) = [0,0,0,0];
jointsAngles(2,:) = [0,5,45,0];
jointsAngles(3,:) = [0,-40,55,0]; 

the1 = [0, 0, 0];
the2 = [0, deg2rad(15), deg2rad(-40)];
the3 = [0, deg2rad(45), deg2rad(55)];

time = zeros(1, 3);
j1 = zeros(1, 3);
j2 = zeros(1, 3);
j3 = zeros(1, 3);
startTime = tic;

j1l = zeros(1, 3);
j2l = zeros(1, 3);
j3l = zeros(1, 3);

c1 = zeros(1,3);
c2 = zeros(1, 3);
c3 = zeros(1, 3);

t1= zeros(1, 3);
t2 = zeros(1, 3);
t3 = zeros(1, 3);

cur1= zeros(1, 3);
cur2 = zeros(1, 3);
cur3 = zeros(1, 3);

tor1= zeros(1, 3);
tor2 = zeros(1, 3);
tor3 = zeros(1, 3);

i = 1; 
for j = 1:3
    robot.moveToSamePos(1, jointsAngles(j,:), 1000); %(1000 is the time in miliseconds

    th = robot.read_joint_vars(true, false);
    cr = robot.readCurrent();
    t = robot.readTorques();

    cur1(j) = cr(1); %current joint 1
    cur2(j) = cr(2); %current joint 2
    cur3(j) = cr(3); %current joint 3

    tor1(j) = t(1); %torque joint 1
    tor2(j) = t(2); %torque joint 2
    tor3(j) = t(3); %torque joint 3

    [I_xx1, I_xy1, I_xz1, I_yx1, I_yy1, I_yz1, I_zx1, I_zy1, I_zz1] = deal(0);
    [I_xx2, I_xy2, I_xz2, I_yx2, I_yy2, I_yz2, I_zx2, I_zy2, I_zz2] = deal(0);
    [I_xx3, I_xy3, I_xz3, I_yx3, I_yy3, I_yz3, I_zx3, I_zy3, I_zz3] = deal(0);
    
    theta_1 = the1(j);
    theta_2 = the2(j);
    theta_3 = the3(j);
    
    c1(j) = cur1(j);
    c2(j) = cur2(j);
    c3(j) = cur3(j);

    t1(j) = tor1(j);
    t2(j) = tor2(j);
    t3(j) = tor3(j);

    DLagrangeAns = subs(D);
    CLagrangeAns = subs(Cnew);
    gravLagrangeAns = subs(gravnew);
    
    DNewtonAns = subs(Dnewton);
    CNewtonAns = subs(Cnewton);
    gravNewtonAns = subs(gravitynewton);
    pause(2);
    gravNewtonAns = subs(gravnew);
    elapsedTime = toc(startTime);
    time(j) = elapsedTime;

    j1(j) = gravNewtonAns(1);
    j2(j) = gravNewtonAns(2);
    j3(j) = gravNewtonAns(3);

    j1l(j) = gravLagrangeAns(1);
    j2l(j) = gravLagrangeAns(2);
    j3l(j) = gravLagrangeAns(3);
end

% Plot the Simulated Torque with respect to time Newton-Euler
figure;
plot(time, j1, '-o', 'DisplayName', 'Joint 1');
hold on;
plot(time, j2, '-o', 'DisplayName', 'Joint 2');
plot(time, j3, '-o', 'DisplayName', 'Joint 3');
xlabel('Time (seconds)');
ylabel('Torques');
title('Newton-Euler Method at 3 Positions');
legend;
grid on;
hold off;

% Plot the Simulated Torque with respect to time Lagrangian Method
figure;
plot(time, j1l, '-o', 'DisplayName', 'Joint 1');
hold on;
plot(time, j2l, '-o', 'DisplayName', 'Joint 2');
plot(time, j3l, '-o', 'DisplayName', 'Joint 3');
xlabel('Time (seconds)');
ylabel('Torques');
title('Lagrangian Method at 3 Positions' );
legend;
grid on;
hold off;

% Plot the Real Current with respect to time 
figure;
plot(time, c1, '-o', 'DisplayName', 'Joint 1');
hold on;
plot(time, c2, '-o', 'DisplayName', 'Joint 2');
plot(time, c3, '-o', 'DisplayName', 'Joint 3');
xlabel('Time (seconds)');
ylabel('Current (mA)');
title('Current at 3 Positions');
legend;
grid on;
hold off;


% Plot the Real Current with respect to time 
figure;
plot(time, t1, '-o', 'DisplayName', 'Joint 1');
hold on;
plot(time, t2, '-o', 'DisplayName', 'Joint 2');
plot(time, t3, '-o', 'DisplayName', 'Joint 3');
xlabel('Time (seconds)');
ylabel('Torque (Nm)');
title('Torque at 3 Positions');
legend;
grid on;
hold off;

disp("Newton Nums:");
disp([j1, j2, j3]);
disp("Lagrange Nums");
disp([j1l, j2l, j3l]);
disp("Current Nums");
disp([c1, c2, c3]);
disp("Torque Nums");
disp([t1, t2, t3]);



function T = FKinSpace(M,Slist,thetalist)
    
    I = eye(3);
    
    for index = 1:length(thetalist)
        if Slist(1:3,index) == [0;0;0]
            theta = thetalist(index);
            ES(:,:,index) = [[I;0,0,0],[Slist(4:6,index)*theta;1]];
        else
            w = [0, -Slist(3,index), Slist(2,index)
                 Slist(3,index), 0, -Slist(1,index)
                 -Slist(2,index), Slist(1,index), 0];
            theta = thetalist(index);
            R = I + (sin(theta)*w) + ((1-cos(theta))*(w^2));
            V = (I*theta + (1-cos(theta))*w + (theta-sin(theta))*w^2) * Slist(4:6,index);
            ES(:,:,index) = [[R;0,0,0], [V;1]];
        end
    end
    
    T = ES(:,:,length(thetalist))*M;
    for index2 = length(thetalist)-1:-1:1
        T = ES(:,:,index2)*T;
    end
     
end