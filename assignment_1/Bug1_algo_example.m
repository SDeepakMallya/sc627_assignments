%function Environment()

clc;
clear all;

%% % input file 
% ........................
% READ ME
% ........................
%     1.obstacle polygon vertices are numbered in clock-wise direction
% 
%     2.number 1 vertex will be the down most point of the side where the
%       will collide
..................................................................................................................
s=0.1;  % s-step_size
t=0.1;  %t-tolerence
p_start= [0 0];
p_goal=[5 3];
obs1=[1 0;1 2;3 0];
obs2=[4 1;2 3;5 2];
xboundary=[0 5];
yboundary=[0 3];
status='no path exsist';

%% % environment plotting
...............................................................................................................
x_p=linspace(xboundary(1,1),xboundary(1,2),0.01);
y_p=linspace(yboundary(1,1),yboundary(1,2),0.01);
plot(x_p,y_p)

xlabel('x-axis')
xlim(xboundary)
ylabel('y-axis')
ylim(yboundary)
hold on;
grid on;


%plotting p_start and p_goal
plot(p_goal(1,1),p_goal(1,2),'*')
plot(p_start(1,1),p_start(1,2),'*')

%plotting obstacle 1
obs1_plot=polyshape([obs1(1,1) obs1(2,1) obs1(3,1)],[obs1(1,2) obs1(2,2) obs1(3,2)]);
plot(obs1_plot)

%plotting obstacle 2
obs2_plot=polyshape([obs2(1,1) obs2(2,1) obs2(3,1)],[obs2(1,2) obs2(2,2) obs2(3,2)]);
plot(obs2_plot)


%%
%obstacle 1 polygon line equations
.....................................................................................................................
coefficients = polyfit([obs1(1,1),obs1(2,1)], [obs1(1,2),obs1(2,2)], 1);
m(1,1) = atand(coefficients (1)); %gradient

coefficients = polyfit([obs1(2,1),obs1(3,1)], [obs1(2,2),obs1(3,2)], 1);
m(1,2) =atand(coefficients (1)); %gradient

coefficients =polyfit([obs1(3,1),obs1(1,1)], [obs1(3,2),obs1(1,2)], 1);
m(1,3) =atand(coefficients (1)); %gradient

%obstacle 1 polygon line equations
.....................................................................
coefficients = polyfit([obs2(1,1),obs2(2,1)], [obs2(1,2),obs2(2,2)], 1);
m(2,1) = atand(coefficients (1)); %gradient

coefficients = polyfit([obs2(2,1),obs2(3,1)], [obs2(2,2),obs2(3,2)], 1);
m(2,2) = atand(coefficients (1)); %gradient

coefficients =polyfit([obs2(3,1),obs2(1,1)], [obs2(3,2),obs2(1,2)], 1);
m(2,3) = atand(coefficients (1)); %gradient


%%
%bug1 path planning
......................................................................................................................
x=p_start(1,1);
y=p_start(1,2);
distance(1)=sqrt((x-p_goal(1,1))^2-(y-p_goal(1,2))^2);
theta(1)=atand((x-p_goal(1,2))/(y-p_goal(1,1)));

%calculating p_hit1
L1=[x  y;p_goal(1,1) p_goal(1,2)];
[inter1] =intersect(obs1_plot,L1);  %inter1=phit1


while distance(1) > s 

   x=x+s*cosd(theta(1));
   y=y+s*sind(theta(1));
      
   plot(x,y,'.')
   hold on
   drawnow 

   distance(1)=sqrt((x-p_goal(1,1))^2+(y-p_goal(1,2))^2);
   dis_obs(1)=sqrt((x-inter1(1,1))^2+(y-inter1(1,2))^2);
  
   if dis_obs(1) < t
     %bug1_circumnavigate()
     break
   elseif x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
       text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
   end
end


%%
%circumnavigate obs1
..........................................................................................................

while   y < obs1(2,2) && y >obs1(1,2) 

   plot(x,y,'.')
   hold on
   drawnow 

   x=x+s*cosd(m(1,1));
   y=y+s*sind(m(1,1));
  
end

if x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
       text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
      
end

%%
%bug1 path after leaving obs1
.....................................................................................................................
distance(2)=sqrt((x-p_goal(1,1))^2-(y-p_goal(1,2))^2);
theta(2)=atand((y-p_goal(1,2))/(x-p_goal(1,1)));

%calculating p_hit2
L2=[x  y;p_goal(1,1) p_goal(1,2)];
[inter2] =intersect(obs2_plot,L2);


while distance(2) > s 
   plot(x,y,'.')
   hold on
   drawnow 

   x=x+s*cosd(theta(2));
   y=y+s*sind(theta(2));
     
   distance(2)=sqrt((x-p_goal(1,1))^2+(y-p_goal(1,2))^2);
   dis_obs(2)=sqrt((x-inter2(1,1))^2+(y-inter2(1,2))^2);
  
   if dis_obs(2) < t  
        break
   elseif x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
       text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
   end
end


%%
%circumnavigate obs2
..........................................................................................................

while y < obs2(2,2) && y >obs2(1,2)  

   x=x+s*cosd(m(2,1));
   y=y+s*sind(m(2,1));

   plot(x,y,'.')
   hold on
   drawnow 
  
end

if x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
      text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
      
end

%%
%bug1 path after leaving obs1
.....................................................................................................................
distance(3)=sqrt((x-p_goal(1,1))^2-(y-p_goal(1,2))^2);
theta(3)=atand((y-p_goal(1,2))/(x-p_goal(1,1)));

%calculating p_hit2
L3=[x  y;p_goal(1,1) p_goal(1,2)];
[inter3] =intersect(obs2_plot,L2);


while distance(2) > s 
   plot(x,y,'.')
   hold on
   drawnow 

   x=x+s*cosd(theta(3));
   y=y+s*sind(theta(3));
     
   distance(3)=sqrt((x-p_goal(1,1))^2+(y-p_goal(1,2))^2);
   dis_obs(2)=sqrt((x-inter3(1,1))^2+(y-inter3(1,2))^2);
  
   if dis_obs(2) < t  
        break
   elseif x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
        text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
        break
   end
end

