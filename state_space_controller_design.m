% State Space Controller Design  

clear all;
close all;
clc;
% state space model of ball balanced by a magnetic field
%-------------------------------------------------------------------------------------------%
                            % State Space Model For Ball Balancing
%-------------------------------------------------------------------------------------------%
m=0.05;
h=0.01;
i=7;
R=1;
g=9.81;
K=0.0001;
L=0.01;

A=[0 1 0;g/h 0 -(K*i)/(m*h);0 0 -R/L];
B=[0;0;1/L];
C=[1 0 0];
D=[0];
sys=ss(A,B,C,D);
%eig(A) %one unstable pole 31.3209
% open loop response for non zero initial condition and no input 

x0=[0.01 ; 0 ; 0];
t=0:0.01:2;
u=zeros(size(t));
%[y,t,x]=lsim(sys,u,t,x0);
%plot(t,y)
%title('openloop response to non zero intial condition zero input');
%xlabel('time');
%ylabel('position');

%-------------------------------------------------------------------------------------------%
                                 % Controllability 
%-------------------------------------------------------------------------------------------%

Ctheta=ctrb(A,B);
rank(Ctheta);

%rank of controllability matric is equal to number of states hence
%controllable for some u 

%-------------------------------------------------------------------------------------------%
                                  % Observability
%-------------------------------------------------------------------------------------------%

Otheta=obsv(A,C);
rank(Otheta);

% rank of observability matrix equal to number of states hence observable
% from current y ie measuring only height(h)

%-------------------------------------------------------------------------------------------%
                        % POLE PLACEMENT METHOD u=-Kx
%-------------------------------------------------------------------------------------------%

p1=-50+10i;
p2=-50-10i;
p3=-100;
new_poles=[p1,p2,p3];
K=place(A,B,new_poles);
sys_pole_placement=ss(A-B*K,B,C,D);
%lsim(sys_pole_placement,u,t,x0);
%title('pole placement');
%xlabel('time');
%ylabel('position');
%eig(sys_pole_placement)

%-------------------------------------------------------------------------------------------%
                                % LQR OPTIMISED CONTROL
%-------------------------------------------------------------------------------------------%

Q=zeros(3);
Q(1,1)=10;
Q(2,2)=10;
Q(3,3)=1;
R=10;
K=lqr(A,B,Q,R);
sys_lqr=ss(A-B*K,B,C,D);
eig(sys_lqr);
%pzmap(sys_lqr)
%lsim(sys_pole_placement,u,t,x0);
%title('LQR');
%xlabel('time');
%ylabel('position');

%-------------------------------------------------------------------------------------------%
                            % Kalman Filter Using LQR Command
%-------------------------------------------------------------------------------------------%

% new system considering disturbance(d) in system and noise(n) in y 
% Xdot = A*x + B*u + d*Vd
% y= C*x + n*Vn

Vd=0.1*eye(3);
Vn=1;
BF=[B Vd 0*B];
D=[0 0 Vn];
Kf=(lqr(A',C',Vd,Vn))';
sysC=ss(A,BF,C,[0 0 0 0 Vn]);
sysFull=ss(A,BF,eye(3),zeros(3,size(BF,2)));
sysKf=ss(A-Kf*C,[B Kf],eye(3),0*[B Kf]);












