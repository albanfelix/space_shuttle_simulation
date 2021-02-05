function X=navettecontinue(A,B,x,t)
tstep=1;
u=0.05*(t>=tstep);
X=A*x+B*u;
end