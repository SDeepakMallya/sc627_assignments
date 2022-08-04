clc
clear
close all

%% Given Data
n=8;                                % Number of robots
VMax=0.15;
V=VMax/sqrt(2);
k=0.2;

%% Robot position
robot(1).position=[0 0];            % Fixed left robot
robot(2).position=[-3 3];
robot(3).position=[3 7];
robot(4).position=[2 -7];
robot(5).position=[4 -4];
robot(6).position=[4 5];
robot(7).position=[17 -6];
robot(8).position=[14 0];           % Fixed right robot

%% Generating the plot
hold on
axis([-5 20 -10 20]) 

% Plotting fixed robots....................................................
plot(robot(1).position(1,1),robot(1).position(1,2),'square','Color','r');
text(robot(1).position(1,1)+0.2,robot(1).position(1,2)+0.2,num2str(1));

plot(robot(8).position(1,1),robot(8).position(1,2),'square','Color','r');
text(robot(8).position(1,1)+0.2,robot(8).position(1,2)+0.2,num2str(8));
%..........................................................................

% Plotting starting position of the moving robots..........................
for i=2:n-1
    plot(robot(i).position(1,1),robot(i).position(1,2),'o','Color','b');
    text(robot(i).position(1,1)+0.2,robot(i).position(1,2)+0.2,num2str(i));
    robot(i).distanceL=sqrt((robot(i).position(1,1)-robot(i-1).position(1,1))^2+(robot(i).position(1,2)-robot(i-1).position(1,2))^2);
    robot(i).distanceR=sqrt((robot(i).position(1,1)-robot(i+1).position(1,1))^2+(robot(i).position(1,2)-robot(i+1).position(1,2))^2);
end
%..........................................................................

%% Balancing based on attraction
for itr=1:10/VMax
    for i=2:n-1

        if  abs(robot(i).position(1,2)-robot(1).position(1,2)) < 0.15
            robot(i).position(1,2)=robot(1).position(1,2);
            Fy(i)=0;
        elseif robot(i).position(1,2)<0
            robot(i).position(1,2)=robot(i).position(1,2)+V;
        else
            robot(i).position(1,2)=robot(i).position(1,2)-V;
        end

        if robot(i).position(1,1) < robot(1).position(1,1)
            robot(i).position(1,1) = robot(i).position(1,1) +V;
        elseif robot(i).position(1,1) > robot(8).position(1,1)
            robot(i).position(1,1) = robot(i).position(1,1) - V;
        end

        robot(i).distanceX(1)=abs(robot(i).position(1,1)-robot(i-1).position(1,1));
        robot(i).distanceX(2)=abs(robot(i).position(1,1)-robot(i+1).position(1,1));
%         robot(i).forceX = robot(i).distanceX(1) - robot(i).distanceX(2);

        if robot(i).distanceX(1) < robot(i).distanceX(2) 
            robot(i).position(1,1)=robot(i).position(1,1) +V;
        elseif robot(i).distanceX(1) > robot(i).distanceX(2)
            robot(i).position(1,1)=robot(i).position(1,1)-V;
        end
        
        robot(i).distanceL=sqrt((robot(i).position(1,1)-robot(i-1).position(1,1))^2+(robot(i).position(1,2)-robot(i-1).position(1,2))^2);
        robot(i).distanceR=sqrt((robot(i).position(1,1)-robot(i+1).position(1,1))^2+(robot(i).position(1,2)-robot(i+1).position(1,2))^2);
        
        robot(i).force=abs(robot(i).distanceL-robot(i).distanceR);

        plot(robot(i).position(1,1),robot(i).position(1,2),'.');
        A(itr,1,i)=itr;
        A(itr,2,i)=robot(i).position(1,1);
        A(itr,3,i)=robot(i).position(1,2);
    end
end


for i=2:n-1
    plot(robot(i).position(1,1),robot(i).position(1,2),'*','Color','r');
    text(robot(i).position(1,1)+0.2,robot(i).position(1,2)+0.2,num2str(i));
end

plot(10,19,'o','Color','b');
text(10.5,19,'- Moving robot at the begining');
plot(10,18,'*','Color','r');
text(10.5,18,'- Final position of the robot');
plot(10,17,'square','Color','r');
text(10.5,17,'- Fixed robots');

xlabel('X-axis');
ylabel('Y-axis');
