close all
clear

%% robot modeling
lx = 1; lr = 0.1; gy = 9.81; fvis = 0; fcou = 0; 
% rod = Cylinder(lr,lx);
% Rcyl = SO3.rpy([0 -pi/2 0]);
% Irod = Rcyl.R*rod.inertia*Rcyl.R';
rod = Cuboid([lx,lr,lr]);
Irod = rod.inertia;
dpm = {'a', lx, 'm', rod.mass, 'r', [-lx/2,0,0], 'qlim', [-pi/2, pi/2],'I', Irod,...
    'B', fvis, 'Tc', [fcou -fcou]};
r = SerialLink([Revolute(dpm{:}),Revolute(dpm{:})],'name','two-link','gravity',[0 gy 0]);

%% test calculation
qz = [pi/6,pi/4]*0;
qd = [0.1,0.2]*0;
I1 = Irod(3,3);
I2 = I1;
m1 = rod.mass;
m2 = m1;
l1 = lx;
l2 = l1;
r1 = -l1/2;
r2 = r1;
c1 = I1+m1*r1^2+I2+m2*(l1^2+r2^2);
c2 = -m2*l1*r2;
c3 = I2+m2*r2^2;
% inertia and coriolis
M = [c1+2*c2*cos(qz(2)),c3+c2*cos(qz(2));
    c3+c2*cos(qz(2)),c3];
C = [-c2*sin(qz(2))*qd(2),-c2*sin(qz(2))*sum(qd);
    c2*sin(qz(2))*qd(1),0];
% friction
F = -fvis*qd'-fcou*sign(qd');
% gravity
G = [-m1*gy*r1*cos(qz(1))+m2*gy*l1*cos(qz(1))-m2*gy*r2*cos(sum(qz));
    -m2*gy*r2*cos(sum(qz))];

% payload
mp = 1;
taup = [mp*gy*(l1*cos(qz(1))+l2*cos(sum(qz)));mp*gy*l2*cos(sum(qz))];
taug = G+taup;

%% my simulation function
y0 = [qz,qd]'; tspan = [0 10]; 
% ydfun = @(t,y,r) [y(r.n+1:end);r.inertia(y(1:r.n)')^-1*(zeros(r.n,1)-r.coriolis(y')*y(r.n+1:end)-r.gravload(y(1:r.n)')')];
% tic
% [tlist,ylist] = ode15s(@(t,y) ydfun(t,y,r),tspan,y0);
% toc

%% free drop in gravity
tic
[tlist,ylist] = ode15s(@(t,y) [y(r.n+1:end);r.accel(y(1:r.n)',y(r.n+1:end)',zeros(1,r.n))],tspan,y0);
toc

%% stay still
% tic
% [tlist,ylist] = ode15s(@(t,y) [y(r.n+1:end);r.accel(y(1:r.n)',y(r.n+1:end)',r.rne(y(1:r.n)',y(r.n+1:end)',zeros(1,r.n)))],tspan,y0);
% toc

%% visualize two link
ws = [-4 4 -4 4 -4 4];
plotopt = {'workspace', ws, 'nobase', 'notiles', 'noshading', 'noshadow', 'nowrist','top'};
h = r.plot(ylist(:,1:r.n),plotopt{:});
