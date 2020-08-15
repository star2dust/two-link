close all
clear

syms th1 th2 r1 r2 l1 l2 dth1 dth2 real
x1 = r1*cos(th1);
dx1 = -r1*sin(th1)*dth1;
y1 = r1*sin(th1);
dy1 = r1*cos(th1)*dth1;
x2 = l1*cos(th1)+r2*cos(th1+th2);
dx2 = -(l1*sin(th1)+r2*sin(th1+th2))*dth1-r2*sin(th1+th2)*dth2;
y2 = l1*sin(th1)+r2*sin(th1+th2);
dy2 = (l1*cos(th1)+r2*cos(th1+th2))*dth1+r2*cos(th1+th2)*dth2;

T = dx1^2+dy1^2+dth1^2+dx2^2+dy2^2+dth2^2;

syms a b c
T2 = [dth1 dth2]*[a+2*b*cos(th2),c+b*cos(th2);
    c+b*cos(th2),c]*[dth1 dth2]';