
function dX = EOM(t, X)
dX = zeros(4,1);
da = 1;
x  = X(1);
vx = X(2);
y  = X(3);
vy = X(4);
dX = [vx; da*(rand()-0.5); vy; da*(rand()-0.5)];
end