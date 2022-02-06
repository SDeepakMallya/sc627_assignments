%function Environment()

clc;
clear all;

%%
%input file
..................................................................................................................
s=0.1;  % s-step_size
p_start= [0 0];
p_goal=[5 3];
obs1=[1 0;1 2;3 0];
obs2=[4 1;2 3;5 2];
xboundary=[0 5];
yboundary=[0 3];
status='no path exsist'

%%
%environment plotting
...............................................................................................................
x_p=linspace(0,5,0.01);
y_p=linspace(0,3,0.01);

% plotting the environment
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
A1=[-3 5;1 0]; b1=[0;1];
p_hit1=A1\b1;

while distance(1) > 0.2 

   x=x+s*cosd(theta(1));
   y=y+s*sind(theta(1));
      
   plot(x,y,'.')
   hold on
   drawnow 

   distance(1)=sqrt((x-p_goal(1))^2+(y-p_goal(2))^2);
   dis_obs(1)=sqrt((x-p_hit1(1))^2+(y-p_hit1(2))^2);
  
   if dis_obs(1) < 0.15 
     %bug1_circumnavigate()
     break
   elseif x>5 || x<0 || y<0 || y>3
        status
   end
end

%%
%circumnavigate obs1
..........................................................................................................

while y < obs1(2,2)
   plot(x,y,'.')
   hold on
   drawnow 

   x=x+s*cosd(m(1,1));
   y=y+s*sind(m(1,1));
  
end

%%
%bug1 path after leaving obs1
%......................................................................................................................
distance(2)=sqrt((x-p_goal(1,1))^2-(y-p_goal(1,2))^2);
theta(2)=atand((y-p_goal(1,2))/(x-p_goal(1,1)));

%calculating p_hit2
A2=[-1 5;1 1]; b2=[9;5];
p_hit2=A2\b2;

while distance(2) > 0.1 
   plot(x,y,'.')
   hold on
   drawnow 

   x=x+s*cosd(theta(2));
   y=y+s*sind(theta(2));
     
   distance(2)=sqrt((x-p_goal(1,1))^2+(y-p_goal(1,2))^2);
   dis_obs(2)=sqrt((x-p_hit2(1))^2+(y-p_hit2(2))^2);
  
   if dis_obs(2) < 0.2  
     
     break
   elseif x>5 || x<0 || y<0 || y>3
        status
   end
end


%%
%circumnavigate obs2
..........................................................................................................

while y < obs2(2,2)

   x=x+s*cosd(m(2,1));
   y=y+s*sind(m(2,1));

   plot(x,y,'.')
   hold on
   drawnow 

   if x>5 || x<0 || y<0 || y>3
    break
   end
  
end

status

