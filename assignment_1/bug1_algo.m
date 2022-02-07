% function s=bug1_algo()
clear all
........................................................................................................................
% p_start=input('Enter the starting point in matrix form.(Eg: [x y z])..........P_start=');
% p_goal=input('Enter the goal point in matrix form.(Eg: [x y z])..........P_goal=');
% s=input('What is the step size.(Eg: 0.1)..........Step size=');
% t=input('What is the tolerance.(Eg: 0.1)..........Tolerance=');
% xboundary=input('what is the x-asxis boundary?[Eg: [Xmin  Xmax].........');
% yboundary=input('what is the y-asxis boundary?[Eg: [Ymin  Ymaxy].........');
% 
% n=input('How many obstacles are there?(Eg: 2)..........number of obstacles=');
% 
% for i=1:n
%     v=input('How many vertices are there?(Eg:3)........number of vertices/sides=');
%     obstacle_number=i
%     for j=1:v
%         obs(j,:,i)=input('Give vertices point .(Eg: [x1 y1])..........');
%     end
%     
% end

s=0.1;  % s-step_size
t=0.2;  %t-tolerence
p_start= [0 0];
p_goal=[5 3];
obs(:,:,1)=[1 0;1 2;3 0];
obs(:,:,2)=[4 1;2 3;5 2];
xboundary=[0 5];
yboundary=[0 3];
n=2;
v=3;

q=1000;

........................................................................................................................
%% %environment plotting
...............................................................................................................
x_p=linspace(xboundary(1,1),xboundary(1,2),0.01);
y_p=linspace(yboundary(1,1),yboundary(1,2),0.01); plot(x_p,y_p)

xlabel('x-axis')
xlim(xboundary) 
ylabel('y-axis') 
ylim(yboundary) 
hold on; 
grid on;

%plotting p_start and p_goal plot(p_goal(1,1),p_goal(1,2),'*')
plot(p_start(1,1),p_start(1,2),'*')

for i=1:n  %number of obstacles
    for j=1:v   %number of vertices
        x_vertex(1,j)=obs(j,1,i); 
        y_vertex(1,j)=obs(j,2,i);
        
    end 
    obs_plot(i)=polyshape(x_vertex,y_vertex); 
    plot(obs_plot(i))
end

%% %obstacle polygon line equations
.....................................................................................................................

for i=1:n   %number of obstacles
    for j=1:v   %number of vertices
%         x_vert=[obs(j,1,i) obs(j+1,1,i)];
%         y_vert=[obs(j,2,i) obs(j+1,2,i)];

        obs(v+1,1,i)=obs(1,1,i);
        obs(v+1,2,i)=obs(1,2,i);
        
        coefficient= polyfit([obs(j,1,i),obs(j+1,1,i)], [obs(j,2,i),obs(j+1,2,i)], 1); %need correction.................
        m(1,j,i)=atand(coefficient(1));
        % row-i th obstacle 
        % column-gradient of the j th side
    end
     
end

%% % shortes distance
for i=1:n
    for j=1:v
        a=[obs(j,1,i),obs(j,2,i),0] - [obs(j+1,1,i),obs(j+1,2,i),0];
        b=[p_goal(1,1) p_goal(1,2) 0] - [obs(j+1,1,i),obs(j+1,2,i),0];
        d(1,j,i) =  norm(cross(a,b)) / norm(a);
    end
    
end

%%

x=p_start(1,1);
y=p_start(1,2);

for i=1:n  %number of obstacles
i=1;
    distance=sqrt((x-p_goal(1,1))^2-(y-p_goal(1,2))^2);
    theta(1)=atand((x-p_goal(1,2))/(y-p_goal(1,1)));

    %calculating p_hit1
    L=[x  y;p_goal(1,1) p_goal(1,2)];
    inter =intersect(obs_plot(1,i),L);  %inter1=phit1
   
    while distance > s
               plot(x,y,'*')
               hold on
               drawnow        

               x=x+s*cosd(theta(1));
               y=y+s*sind(theta(1));
               
               distance = sqrt((x-p_goal(1,1))^2+(y-p_goal(1,2))^2);
               dis_obs = sqrt((x-inter(1,1))^2+(y-inter(1,2))^2);
              
               if dis_obs < t
                 break
               elseif x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
                   text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
               end
    end
                   

    for j=1:v  %numver of vertices
        %circumnavigate obs
       
            if obs(j,2,i) > obs(j+1,2,i)

                if obs(j,1,i) > obs(j+1,1,i)
                    
                    while   y < obs(j,2,i)+s && y > obs(j+1,2,i)-s &&  x < obs(j,1,i)+s && x > obs(j+1,1,i)-s
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));          
                    end
                elseif obs(j,1,i) <= obs(j+1,1,i)
                    
                    while   y < obs(j,2,i)+s && y > obs(j+1,2,i)-s &&  x > obs(j,1,i)-s && x < obs(j+1,1,i)+s
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,1,i));
                       y=y+s*sind(m(1,1,i));          
                    end
                else
                    while   y < obs(j,2,i)+s && y > obs(j+1,2,i)-s 
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));   
                    end
                end
            end
                  
          
            if obs(j,2,i) < obs(j+1,2,i)
                 
                if obs(j,1,i) > obs(j+1,1,i)
                    
                    while   y > obs(j,2,i)-s && y < obs(j+1,2,i)+s &&  x < obs(j,1,i)+s && x > obs(j+1,1,i)-s
                       
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));          
                    end
                elseif obs(j,1,i) < obs(j+1,1,i)
                    
                    while   y > obs(j,2,i)-s && y < obs(j+1,2,i)+s &&  x > obs(j,1,i)-s && x < obs(j+1,1,i)+s
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));          
                    end
                elseif obs(j,1,i) == obs(j+1,1,i)
                     
                    while   y > obs(j,2,i)-s && y < obs(j+1,2,i)+s 
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));   
                    end
                    
                end
            end

            if obs(j,2,i) == obs(j+1,2,i)
                if obs(j,1,i) > obs(j+1,1,i)
                    
                    while  x < obs(j,1,i)+s && x > obs(j+1,1,i)-s 
                       
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));          
                    end
                elseif obs(j,1,i) < obs(j+1,1,i)
                    
                    while    x > obs(j,1,1)-s && x < obs(j+1,1,1)+s
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));          
                    end
                elseif obs(j,1,i)==obs(j+1,1,i)
                    while   1
                       plot(x,y,'o')
                       hold on
                       drawnow 
                    
                       x=x+s*cosd(m(1,j,i));
                       y=y+s*sind(m(1,j,i));   
                    end
                end
            end
                
                
             if x>xboundary(1,2) || x<xboundary(1,1) || y<yboundary(1,1) || y>yboundary(1,2)
                       text(((xboundary(1,2)-xboundary(1,1))/2)-0.5,(yboundary(1,2)-yboundary(1,1))/2 , status);
                      
             end
                
    

    end
    
    
end
