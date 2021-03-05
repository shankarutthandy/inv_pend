% RLC transfer function
clear all;
clc;
R=10;
L=0.0001;
c=0.001;
num=[1];
den=[L R c];
transfer_func=tf(num,den)
%step(transfer_func)

pole(transfer_func)
pzmap(transfer_func)