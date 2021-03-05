%mass spring damper stae space
m=1;
b=.001;
k=0.1;
F=1;
A=[0 1;-k/m -b/m];
B=[0;1/m];
C=[1 0;0 0];
D=[0;0];
sys=ss(A,B,C,D);

det(ctrb(A,B)); %det != 0 hence the system is controllable 
eigs(A,B); %eigs real part negative hence a stable system