function Xe=equarec(Atilde,Btilde,n,xe,ts)
tstep=1;
u=0.05*(n>=tstep/ts);
Xe=Atilde*xe+Btilde*u;
end
