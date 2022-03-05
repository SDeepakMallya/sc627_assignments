clc
clear
close all

%% ......................Artificial Potential Field.........................
syms x1 x2 y1 y2

p_start              = [0 0]               ;% Rabot Coordinate = [x-axis y-axis]
obs1                 = [1 0;1 2;3 0;1 0.5;1 1.5;1.5 0;2.5 0;2 1;2.5 0.5;1.5 1.5]       ;% Obstacle1 Coordinate = [x-axis y-axis]
obs2                 = [4 1;2 3;5 2;2.5 2.5;3 2;3.7 1.2;4.5 1.5;4.5 2.2;3.5 2.5;2.8 2.8]       ;% Obstacle2 Coordinate = [x-axis y-axis]
p_goal               = [5 3]               ;% Target Coordinate = [x-axis y-axis]
Step_Size            = 0.1                 ;% Radius of Circle for Artificial Points
Obstacle             = 0.8                 ;% Obstacle = Alpha Obstacle = 0.8
Q_rep_obs            = 2                   ;% Q*_obs = 2
Goal                 = 0.8                 ;% Target   = Alpha Goal = 0.8
d_att_goal           = 2                   ;% d*_goal = 2
Confirm_Message      = 'Solution Exists'   ;% Message Displayed if a Good Artificial Point Exists
Error_Message        = 'No Solution Exists';% Message Displayed if no Good Artificial Point Exists
tol                  = 0.01                ;% Tolerance

%% ...........................Generating Environment Plot.....................................

hold on
axis([-3 9 -3 9])               

%plotting p_start and p_goal
plot(p_goal(1,1),p_goal(1,2),'o')
plot(p_start(1,1),p_start(1,2),'o')

%plotting obstacle 1
obs1_plot=polyshape([obs1(1,1) obs1(2,1) obs1(3,1)],[obs1(1,2) obs1(2,2) obs1(3,2)]);
plot(obs1_plot)

%plotting obstacle 2
obs2_plot=polyshape([obs2(1,1) obs2(2,1) obs2(3,1)],[obs2(1,2) obs2(2,2) obs2(3,2)]);
plot(obs2_plot)


%% .........................Potential Field Calculations.......................

x0=p_start;


%Calculating the attractive potential field made by goal
DTG         = sqrt((x1-p_goal(1,1))^2 + (x2-p_goal(1,2))^2)    ;%Distance To Goal
DTGx        = inline(DTG);
DTG_ObjFun  = @(x) DTGx(x(1),x(2))                             ;%Gives the gradient value at given point

if DTG_ObjFun(x0) < d_att_goal
    U_goal      = 0.5*Obstacle*DTG^2;
    delt_U_goal = gradient(U_goal);
else
    U_goal      = d_att_goal*Obstacle*DTG - 0.5*Obstacle*d_att_goal^2;
    G_U_goal    = gradient(U_goal);
    Gx_U_goal   = inline(G_U_goal);
    Grad_U_goal = @(x) Gx_U_goal(x(1),x(2));
end

%Calculating the repulsive potential fiels made by obstacle
U_obs = 0;
for v=1:10 %obstacle vertices

%Obstacle 1
    DTO1        = sqrt((x1-obs1(v,1))^2 + (x2-obs1(v,2))^2);      %Distance To obstacle 1
    DTO1x       = inline(DTO1);
    DTO1_ObjFun = @(x) DTO1x(x(1),x(2));  

    if DTO1_ObjFun(x0) < Q_rep_obs
        U_obs1       = 0.5*Goal*((1/DTO1)-(1/Q_rep_obs))^2;
        G_U_obs1     = gradient(U_obs1);
        Gx_U_obs1    = inline(G_U_obs1);
        Grad_U_obs1  = @(x) Gx_U_obs1(x(1),x(2));
    else
        U_obs1       = 0;
        delt_U_obs   = 0;
    end

%Obstacle 2
    DTO2        = sqrt((x1-obs2(v,1))^2 + (x2-obs2(v,2))^2);      %Distance To obstacle 2
    DTO2x       = inline(DTO2);
    DTO2_ObjFun = @(x) DTO2x(x(1),x(2));  

    if DTO2_ObjFun(x0) < Q_rep_obs
        U_obs2       = 0.5*Goal*((1/DTO2)-(1/Q_rep_obs))^2;
        G_U_obs2     = gradient(U_obs2);
        Gx_U_obs2    = inline(G_U_obs2);
        Grad_U_obs2  = @(x) Gx_U_obs2(x(1),x(2));
    else
        U_obs2       = 0;
        delt_U_obs   = 0;
    end
    
%Total repulsive force by both the obstacles
    U_obs = U_obs + U_obs1 + U_obs2 ;

end

%Total potential field on robot
    U       = U_goal + U_obs;

    G_U     = gradient(U) ;
    Gx_U    = inline(G_U);
    Grad_U  = @(x) Gx_U(x(1),x(2));

    H_U     = hessian(U);
    Hx_U    = inline(H_U);
    Hess_U  = @(x) Hx_U(x(1),x(2));

    
%% ...........................Distance to Goal...........................
i=0;
a=[];
b=[];

while abs(Grad_U(x0)) > tol 

    S  = -Grad_U(x0);
    x0 = x0 + (Step_Size*S)';
    i=i+1;
    a=[a,x0(1)];
    b=[b,x0(2)];

    %Calculating the attractive potential field made by goal
    DTG         = sqrt((x1-p_goal(1,1))^2 + (x2-p_goal(1,2))^2)    ;%Distance To Goal
    DTGx        = inline(DTG);
    DTG_ObjFun  = @(x) DTGx(x(1),x(2))                             ;%Gives the gradient value at given point
    
    if DTG_ObjFun(x0) <= d_att_goal
        U_goal      = 0.5*Obstacle*(DTG^2);
        delt_U_goal = gradient(U_goal);
    else
        U_goal      = d_att_goal*Obstacle*DTG - 0.5*Obstacle*(d_att_goal^2);
        G_U_goal    = gradient(U_goal);
        Gx_U_goal   = inline(G_U_goal);
        Grad_U_goal = @(x) Gx_U_goal(x(1),x(2));
    end
    
    %Calculating the repulsive potential fiels made by obstacle
    U_obs = 0;

    for v=1:10 %obstacle vertices
    
    %Obstacle 1
        DTO1        = sqrt((x1-obs1(v,1))^2 + (x2-obs1(v,2))^2);      %Distance To obstacle 1
        DTO1x       = inline(DTO1);
        DTO1_ObjFun = @(x) DTO1x(x(1),x(2));  
    
        if DTO1_ObjFun(x0) <= Q_rep_obs
            U_obs1       = 0.5*Goal*(((1/DTO1)-(1/Q_rep_obs))^2);
            G_U_obs1     = gradient(U_obs1);
            Gx_U_obs1    = inline(G_U_obs1);
            Grad_U_obs1  = @(x) Gx_U_obs1(x(1),x(2));
        else
            U_obs1       = 0;
            delt_U_obs   = 0;
        end
    
    %Obstacle 2
        DTO2        = sqrt((x1-obs2(v,1))^2 + (x2-obs2(v,2))^2);      %Distance To obstacle 2
        DTO2x       = inline(DTO2);
        DTO2_ObjFun = @(x) DTO2x(x(1),x(2));  
    
        if DTO2_ObjFun(x0) <= Q_rep_obs
            U_obs2       = 0.5*Goal*(((1/DTO2)-(1/Q_rep_obs))^2);
            G_U_obs2     = gradient(U_obs2);
            Gx_U_obs2    = inline(G_U_obs2);
            Grad_U_obs2  = @(x) Gx_U_obs2(x(1),x(2));
        else
            U_obs2       = 0;
            delt_U_obs   = 0;
        end
        
    %Total repulsive force by both the obstacles
        U_obs = U_obs + U_obs1 + U_obs2; 

    end

    %Total potential field on robot
    U       = U_goal + U_obs;

    G_U     = gradient(U) ;
    Gx_U    = inline(G_U);
    Grad_U  = @(x) Gx_U(x(1),x(2));

plot(a,b)
end 





