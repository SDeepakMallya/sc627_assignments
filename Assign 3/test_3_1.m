clc
clear
close all

%% Environment data
pStart              = [0 0]               ;% Rabot Coordinate = [x-axis y-axis]
pGoal               = [5 0]               ;% Target Coordinate = [x-axis y-axis]
tspan               = 0:0.01:5            ;% time duration

%% Obstacle data
obs = [2 2];% Obstacle location
obsRad=0.15/2;
robRad=0.15/2;
FinalObsRad=obsRad + robRad;
vMax=0.15;

%% Environment plot
hold on
axis([-2 7 -5 5])

plot(pStart(1),pStart(2),'o','Color','r');
text(pStart(1)+0.2,pStart(2)+0.2,'Start');

plot(pGoal(1),pGoal(2),'o','Color','r');
text(pGoal(1)+0.2,pGoal(2)+0.2,'Goal');

theta=linspace(0,2*pi);
xobs=FinalObsRad*cos(theta) + obs(1);
yobs=FinalObsRad*sin(theta) + obs(2);
plot(xobs,yobs);
axis equal

X0=[0 5 vMax vMax];
[T, Xsol] = ode45(@EOM, tspan, X0);
figure
plot(T, Xsol(:,1))
xlabel('x axis')
ylabel('Time')
title('x vs. t')

% plot(Xsol(:,1), Xsol(:,3))
% xlabel('x-axis')
% ylabel('y-axis')
% title('x vs. y')

xlim([0 5])

