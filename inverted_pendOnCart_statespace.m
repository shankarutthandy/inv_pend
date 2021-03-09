%Inverted Pendulum On a Cart Controlled(LQR) and estimated Using Only poseX
close all;
clear all;
clc;
M = .5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
p = I*(M+m)+M*m*l^2; 
x0=[0.01 ; 0 ; -3.14; 0];
t=0.0:0.01:2;
u=2*ones(size(t));
A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
sys=ss(A,B,C,D);
Q=zeros(4);
Q(1,1)=5000;
Q(3,3)=50;
R=[0.0001];
K=lqr(A,B,Q,R);
J=[1 0 0 0];    
sys_rscale=ss(A,B,J,0);
N=rscale(sys_rscale,K);
sys_precompensated=ss((A-B*K),B*N,C,D);
[y,t,x]=lsim(sys_precompensated,u,t);
C=[1 0 0 0;0 0 0 0];
Ac=A-B*K;
%eig(Ac)
L=place(A',C',[-40,-41,-42,-43])';
Ae=[Ac B*K;zeros(size(A)) A-L*C];
Be=[B*N;zeros(size(B))];
Ce=[C zeros(size(C))];
De=[0;0];
sys_estimated_lqr=ss(Ae,Be,Ce,De);
[y,t,x]=lsim(sys_estimated_lqr,u,t);
W=1.5;
H=0.6;
Y=H/2;
L=1.5;
for i=1:size(t)
    X=x(i,1);
    theta=x(i,3);
    plot([-10,10],[-0.4,-0.4],'k','LineWidth',2),hold on
    rectangle('Position',[X-W/2,Y-H/2,W,H],'Curvature',.1,'FaceColor',[1.0 0.0 0.0],'LineWidth',1.5); 
    wheel1=nsidedpoly(1000,'Center',[X-0.45 -0.2],'Radius',0.2);
    wheel2=nsidedpoly(1000,'Center',[X+0.45 -0.2],'Radius',0.2);
    plot(wheel1,'FaceColor','k');
    plot(wheel2,'FaceColor','k');
    plot([X,X+(L*sin(theta))],[Y+H/2,Y+(L*cos(theta))],'k','LineWidth',2)
    axis([-5 5 -3 3]);
    xlabel('POSITION OF CART')
    drawnow;
    hold off;
end   
close(myvideo);
