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
t=0.01:0.01:3;
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
%lsim(sys,u,t,x0);
%eig(A)
%rank(ctrb(A,B))
% system has unstable poles and is controllable since rank equals 4 
%p1=-10+10i;
%p2=-10-10i;
%p3=-30+20i;
%p4=-30-20i;
%poles=[p1,p2,p3,p4];
%K=place(A,B,poles)
%sys_pp=ss(A-B*K,B,C,D);
%eig(sys_pp);
%lsim(sys_pp,u,t,x0)

% lqr 

Q=zeros(4);
Q(1,1)=5000;
Q(3,3)=100;
R=[1];
K=lqr(A,B,Q,R);
%sys_lqr=ss((A-B*K),B,C,D);
%[y,t,x]=lsim(sys_lqr,u,t);
%plot(t,y(:,1),'r');
%hold on;
%plot(t,y(:,2),'g');

% lqr with precompensation gain outside feedback loop
J=[1 0 0 0];        % we are controlling through 'x' only
sys_rscale=ss(A,B,J,0);
N=rscale(sys_rscale,K);
sys_precompensated=ss((A-B*K),B*N,C,D);
[y,t,x]=lsim(sys_precompensated,u,t);
%plot(t,y(:,1),'r');
%hold on;
%plot(t,y(:,2),'g');

%rank(obsv(A,[1 0 0 0;0 0 0 0]));
%rank(obsv(A,[0 1 0 0;0 0 0 0]));
%rank(obsv(A,[0 0 1 0;0 0 0 0]));
%rank(obsv(A,[1 0 0 1;0 0 0 0]));

% the system us observable using x pose estimator since rank 4
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
%plot(t,x(:,1),'r');
%hold on;
%plot(t,x(:,3),'g');
%legend('position of cart','Angle with vertical');
% the system is estimated and controlled using only 'X pose output'
W=1;
H=0.6;
Y=H/2;
L=1.5;
for i=1:size(t)
    X=x(i,1);
    theta=x(i,3);
    plot([-10,10],[0,0],'k','LineWidth',2),hold on
    rectangle('Position',[X-W/2,Y-H/2,W,H],'Curvature',.1,'FaceColor',[1.0 0.0 0.0],'LineWidth',1.5); 
    plot([X,X+(L*sin(theta))],[Y,Y+(L*cos(theta))],'k','LineWidth',4)
    axis([-5 5 -3 3]);
    drawnow,hold off
end   