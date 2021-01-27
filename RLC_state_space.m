% RLC circuit state space
R=100;
L=0.001;
c=0.1;
A=[0 1;-1/L*c -R/L];
B=[0;1/L];
C=[1 0];
D=[0];
sys=ss(A,B,C,D);
det(ctrb(A,B));
obsv(A,C)   %observability rank ==2 same as number of states hence sys observable with just measuring q(charge on the conductors)
eig(A)
