clc
clear
close all

%% Environment data
pStart= [0 0];      % Rabot Coordinate = [x-axis y-axis]
pGoal = [5 0];      % Target Coordinate = [x-axis y-axis]
tspan = 0:0.01:5;   % time duration
vRob  = 0.1;        % Robot velocity along x-axis

%% Obstacle data
obs(1).position=[2 -1]; % Moving obstacle
obs(2).position=[3 1];  % Moving obstacle
obs(3).position=[4 0];  % Stationary obstacle

obs(1).velocity=[0 0.1];
obs(2).velocity=[0 0.1];
obs(3).velocity=[0 0];

obsRad=0.15/2;
robRad=0.15/2;
FinalObsRad=obsRad + robRad;

%% Current positions initiiation
for i=1:3
    obs(i).current=obs(i).position;
end
robot.current=pStart;

%% obstacle 1&2 motion matrix
%obs1.....................................................................
robot.distance(1)=sqrt((pStart(1)-pGoal(1))^2+(pStart(2)-pGoal(2))^2);
A(1,1,1)=obs(1).current(1);   % x_position
A(1,2,1)=obs(1).current(2);   % y_position
A(1,3,1)=0;                   % Time

for k=2:(2/0.1)+1
   obs(1).current(1)=obs(1).current(1);
   obs(1).current(2)=obs(1).current(2)+0.1;
   A(k,1,1)=obs(1).current(1);    % x position
   A(k,2,1)=obs(1).current(2);    % y position
   A(k,3,1)=A(k-1,3)+0.1;         % time
end
n=k;
for k=n+1:n+(2/0.1)
   obs(1).current(1)=obs(1).current(1);
   obs(1).current(2)=obs(1).current(2)-0.1;
   A(k,1,1)=obs(1).current(1);    % x position
   A(k,2,1)=obs(1).current(2);    % y position
   A(k,3,1)=A(k-1,3)+0.1;         % time
end
n=k;
for k=n+1:n+(2/0.1)
   obs(1).current(1)=obs(1).current(1);
   obs(1).current(2)=obs(1).current(2)+0.1;
   A(k,1,1)=obs(1).current(1);    % x position
   A(k,2,1)=obs(1).current(2);    % y position
   A(k,3,1)=A(k-1,3)+0.1;         % time
end
%obs2.........................................................................
A(1,1,2)=obs(2).current(1);   % x_position
A(1,2,2)=obs(2).current(2);   % y_position
A(1,3,2)=0;                   % Time

for k=2:(2/0.1)+1
   obs(2).current(1)=obs(2).current(1);
   obs(2).current(2)=obs(2).current(2)-0.1;
   A(k,1,2)=obs(2).current(1);    % x position
   A(k,2,2)=obs(2).current(2);    % y position
   A(k,3,2)=A(k-1,3)+0.1;         % time
end
n=k;
for k=n+1:n+(2/0.1)
   obs(2).current(1)=obs(2).current(1);
   obs(2).current(2)=obs(2).current(2)+0.1;
   A(k,1,2)=obs(2).current(1);    % x position
   A(k,2,2)=obs(2).current(2);    % y position
   A(k,3,2)=A(k-1,3)+0.1;         % time
end
n=k;
for k=n+1:n+(2/0.1)
   obs(2).current(1)=obs(2).current(1);
   obs(2).current(2)=obs(2).current(2)-0.1;
   A(k,1,2)=obs(2).current(1);    % x position
   A(k,2,2)=obs(2).current(2);    % y position
   A(k,3,2)=A(k-1,3)+0.1;         % time
end
%robot.......................................................................
B(1,1)=robot.current(1);   % x_position
B(1,2)=robot.current(2);   % y_position
B(1,3)=0;                   % Time

for k=2:(5/0.1)+1
   robot.current(1)=robot.current(1)+0.1;
   robot.current(2)=robot.current(2);
   B(k,1)=robot.current(1);    % x position
   B(k,2)=robot.current(2);    % y position
   B(k,3)=B(k-1,3)+0.1;        % time
end
%% checking the collision

for i=1:length(B)
    if B(i,1)-A(i,1,1)<0.3 && B(i,2)-A(i,2,1)<0.3         % checking collision with obs 1
        B(i,1)=B(i,1);
        B(i,2)=B(i,2)+0.15;
    elseif B(i,1)-A(i,1,2)<0.3 && B(i,2)-A(i,2,2)<0.3     % checking collision with obs 2
        B(i,1)=B(i,1);
        B(i,2)=B(i,2)+0.15;
    elseif B(i,1)-obs(3).current(1)<0.3 && B(i,2)-obs(3).current(2)<0.3    % checking collision with obs 3
        B(i,1)=B(i,1);
        B(i,2)=B(i,2)+0.15;
    else
        B(i,1)=B(i,1);
        B(i,2)=B(i,2)-0.15;
    end
end
%% plot movement  
for i=1:length(B) 
    figure(1)
    xlim([-1 6])
    ylim([-1 1])
    grid on
    viscircles([A(i,1,1) A(i,2,1)],obsRad)
    axis equal
    hold on;
    viscircles([A(i,1,2) A(i,2,2)],obsRad)
    axis equal
    hold on;
    viscircles([obs(3).position(1) obs(3).position(2)],obsRad)
    hold on;
    viscircles([B(i,1) B(i,2)],robRad,'Color','g')
    hold on;
    hold off;
end
