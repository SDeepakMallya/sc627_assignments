function plot_xvst(A,j)
% A -is the table which contain x coordinate in given time
% j -is the robot number

plot(A(:,2,j),A(:,1,j))         % x vs time
% plot(A(:,2,j),A(:,3,j))       % x vs y

xlabel('x coordinates')
ylabel('time')