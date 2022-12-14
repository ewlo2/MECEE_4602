% Affiliation: ROAR @ Columbia
% Date:        12/02/2021

clear
clc
close all

%%
%link Length 
link = [27;5;5];
% Build your robot 
robot = PlanarTwoLink(link);
t = 5;% time to move
n = 50; %number of points 
T = linspace(0,t,n*2); %time array

%inital and final posion 
xi = -2; 
yi = -0;
zi = 5;
xf = 0; %reminder do not have a 0,0 this is a singular and results in NAN: 
yf = 2;
zf = 4.9

%perform IKmotion using a linear motion: plan out the space and change to
%joint space 
% divide the trajectory into 50 equal linear parts

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%METHOD 1(3a)
% X = linspace(xi,xf,n);
% Y = linspace(yi,yf,n);
% q = robot.InverseKinematics(xi, yi)';
% for i = 2 : n
%     q = [q robot.InverseKinematics(X(i), Y(i))'];
% end  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%METHOD 2 (3b)
%perfrom IKmotion using 3rd order interpolation with inital/ final position and velocity 
% get inital Q and interpoate the joint space 
 
%inital velocity 
qIvel = [ 0 0]; 
%final velocity 
qFvel = [0 0];

% qI = robot.InverseKinematics(xi, yi, zi); %perfrom IK for inital position
% qF = robot.InverseKinematics(xf, yf, zf); %perfrom IK for final position

% qI = [-20 0];
% qF1 = [-5 0];
% qF2 = [5, 5];
% qF3 = [2, 10];
qI = [-16 10];
qF1 = [7 5];
qF2 = [3, 0];
qF3 = [0, 0];

%inital velocity 
vI1 = [0 0]; 
vF1 = [0 0];
vF2 = [0 0];
vF3 = [0 0];

% vI1 = [0 0]; 
% vF1 = [0 0];
% vF2 = [0 0];
% vF3 = [0 0];

% vI1 = [0 0]; 
% vF1 = [75 0];
% vF2 = [-20 0];
% vF3 = [-75 0];

t1 = t*0.20;
t2 = t1+t*0.10;
t3 = t2+t*0.10;


[q, v, a] = robot.IKmotion(qI(1), qI(2),qF1(1), qF1(2),vI1(1), vF1(1), vI1(2), vF1(2), 0, t1,n);
[q1, v1, a1] = robot.IKmotion(qF1(1),qF1(2),qF2(1),qF2(2), vF1(1), vF2(1), vF1(2), vF2(2),t1, t2, n);
[q2, v2, a2] = robot.IKmotion(qF2(1),qF2(2),qF3(1),qF3(2), vF2(1), vF3(1), vF2(2), vF3(2),t2, t3, n);


q_b = [q q1 q2];
v_b = [v v1 v2];
a_b = [a a1 a2];


ani = figure;
vid = VideoWriter('Trajectory_Sim'); 
vid.FrameRate = 10;
open(vid)
for i = 1:length(T)
    robot.setJointAngle(q_b(:,i));
    if i < length(T)
        dt = T(i+1)-T(i);
    end
    robot.animateMotion(dt, vid)
end
close(vid)

f = figure;
plot(T, q_b(1,:));
axis([0 3 -30 30]);
xlabel('Gait Cycle [s]')
ylabel('Ankle Angle [deg]')

f = figure;
plot(T, q_b(2,:));
axis([0 3 -5 5]);
xlabel('Gait Cycle [s]')
ylabel('Supination Angle [deg]')

f = figure;
plot(T, v_b(1,:));
axis([0 3 -50 50]);
xlabel('Gait Cycle [s]')
ylabel('Ankle Joint Vel [deg/s]')

f = figure;
plot(T, v_b(2,:));
axis([0 3 -10 10]);
xlabel('Gait Cycle [s]')
ylabel('Supination Joint Vel [deg/s]')

f = figure;
plot(T, a_b(1,:));
axis([0 3 -200 200]);
xlabel('Gait Cycle [s]')
ylabel('Ankle Joint Acceleration [deg/s^2]')

f = figure;
plot(T, a_b(2,:));
axis([0 3 -30 30]);
xlabel('Gait Cycle [s]')
ylabel('Supination Joint Acceleration [deg/s^2]')


