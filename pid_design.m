%PID controller design for spring damp system

% feedback gain = 1 
clear all;
close all;
clc;
m=1;
b=10;
k=20;
F=1;
num=[1];
den=[m b k];
Plant=tf(num,den);
%step(Plant)  ; as seen the open loop system has a
% Steady State Error = 0.95
%pzmap(Plant) % Stable 
%--------------------------------------%
 
% P-controller=> Kp=100 Ki=0 Kd=0
Kp=300;
Controller=pid(Kp);
% New Transfer Function with feedback pid Kp=100
T=feedback(Controller*Plant,1);
%step(T) steady state error reduced to almost 10% with overshoot

%--------------------------------------%

% PD controller=> Kp=300 Kd=50 Ki=0

Kd=50;
Controller=pid(Kp,0,Kd);
T=feedback(Controller*Plant,1);
%step(T); steady state error 10% and no overshoot

%---------------------------------------%

% PI controller => Kp=300 Kd=0 Ki=1

Ki=0.001;
Controller=pid(Kp,Ki,0);
T=feedback(Controller*Plant,1);
%step(T)

%---------------------------------------%

% PID controller => Kp=350 Kd=50 Ki=300

Controller=pid(350,300,50);
T=feedback(Controller*Plant,1);
step(T)
% PID controller is more efficient in this case
