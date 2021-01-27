%spring damp model in tf

m=1;
b=0.001;
k=0.1;
num=[1];
den=[m b k];
transfer_func=tf(num,den);

%step(transfer_func)