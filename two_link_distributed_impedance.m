close all
clear

%% robot modeling
lx = 1; lr = 0.1; g = [0 -9.81 0]; fvis = 0; fcou = 0; 
rod = Cuboid([lx,lr,lr]);
Irod = rod.inertia;
dpm = {'a', lx, 'm', rod.mass, 'r', [-lx/2,0,0], 'qlim', [-pi/2, pi/2],'I', Irod,...
    'B', fvis, 'Tc', [fcou -fcou]};
rob(1) = SerialLink([Revolute(dpm{:}),Revolute(dpm{:})],'name','r1','gravity',...
    -g,'base',SE3([-1,0,0]/2));
rob(2) = SerialLink([Revolute(dpm{:}),Revolute(dpm{:})],'name','r2','gravity',...
    -g,'base',SE3([1,0,0]/2));

%% object modeling
obj = Cuboid([lx,lx,lr]);

%% joint positions (x,y)
x0 = [-1,1.5,0;
    0,1.5,0];
for i=1:2
   q(i,:) = rob(i).ikine(SE3(x0(i,:)),'mask',[1,1,0,0,0,0]); 
end
q(1,:) = [sum(q(1,:)),-q(1,2)];
dq = zeros(size(q));
qd = q;

%% object position (x,y,th)
Xo = [sum(x0)/size(x0,1),0,0,0];
dXo = zeros(1,6);
for i=1:2
   ro(i,:) = x0(i,:)-Xo(1:3);
end

%% visualize systems
ws = [-4 4 -4 4 -4 4];
plotopt = {'workspace', ws, 'nobase', 'notiles', 'noshading', 'noshadow', 'nowrist'};
for i=1:2
    hr(i) = rob(i).plot(q(i,:),plotopt{:}); hold on
end
ho = obj.plot(Xo);
view(2);

%% object trajectory
dXd = [0.2,0,0,0,0,0];
Xd = Xo+rand(size(Xo)).*[1,1,0,0,0,1]/10;
ddXo = @(Xo,dXo,Xd,dXd) -5*(dXo-dXd)-5*(Xo-Xd);
Fint = [0.1,0.1,0,0,0,0.1];

%% object impedance
Mo = blkdiag(obj.mass*eye(3),obj.inertia);
Co = @(dXo) [-obj.mass*g';skew(dXo(4:6))*obj.inertia*dXo(4:6)'];
G = @(Xo) [eye(3),eye(3);
    skew((SO3.rpy(Xo(4:6))*ro(1,:)')'),skew((SO3.rpy(Xo(4:6))*ro(2,:)')')];

%% command force
Fcmd = @(Xo,dXo,Xd) (pinv(G(Xo))*(Co(dXo)+Mo*ddXo(Xo,dXo,Xd,dXd)')+(eye(6)-G(Xo)*pinv(G(Xo)))*Fint')'; 

%% simulation
dt = 0.05;
T = 5;
k = 0;
for t=0:dt:T
    k = k+1;
    % update robot
    F = Fcmd(Xo,dXo,Xd);
    for i=1:2
        md = 3; kv = 15; kp = 25;
        % end-effector trajectory
        J = rob(i).jacob0(q(i,:),dq(i,:)); J(3:6,:)=[];
        Jdq = rob(i).jacob_dot(q(i,:),dq(i,:)); Jdq(3:6,:)=[];
        x(i,:) = rob(i).fkine(q(i,:)).t;
        dx(i,:) = J*dq(i,:)'; 
        xd(i,:) = Xo(1:3)+(SO3.rpy(Xo(4:6))*ro(i,:)')'+F(3*i-2:3*i)/kp;
        dxd = reshape((G(Xo)'*dXo')',[3,2])';
        % joint torque
        Mq = rob(i).inertia(q(i,:)); 
        Cq = rob(i).coriolis(q(i,:),dq(i,:));
        Gq = rob(i).gravload(q(i,:))';
        Jinv = Mq^-1*J'*(J*Mq^-1*J')^-1;
        Mx = Jinv'*Mq*Jinv;
        Cx1 = Jinv'*Cq*Jinv;
        Cx2 = Jinv'*Mq*Jinv*Jdq;
        Gx = Jinv'*Gq;
        tau(i,:) = J'*(-Mx*md^-1*(kv*(dx(i,1:2)-dxd(i,1:2))+kp*(x(i,1:2)-xd(i,1:2)))'+...
            Cx1*dx(i,1:2)'+Cx2+Gx+(eye(2)-Mx*md^-1)*F(3*i-2:3*i-1)');
        ddq(i,:) = rob(i).accel(q(i,:),dq(i,:),tau(i,:)-F(3*i-2:3*i-1)*J);
    end
    dq = dq+dt*ddq;
    q = q+dt*dq;
    for i=1:2
       rob(i).animate(q(i,:)); 
    end
    % update object
    ddXo = (Mo^-1*(G(Xo)*F'-Co(dXo)))';
    dXo = dXo+dt*ddXo;
    Xo = Xo+dt*dXo;
    obj.animate(Xo);
    % update trajectory
    Xd = Xd+dt*dXd;
    drawnow
    % save data
    t_data(k) = t;
    Fcmd_data(k,:) = F;
    tau1_data(k,:) = tau(1,:);
    tau2_data(k,:) = tau(2,:);
    Xo_err(k,:) = Xo-Xd;
    x1_err(k,:) = rob(1).fkine(q(1,:)).t'-(SO3.rpy(Xo(4:6))*ro(1,:)')'-Xo(1:3);
    x2_err(k,:) = rob(2).fkine(q(2,:)).t'-(SO3.rpy(Xo(4:6))*ro(2,:)')'-Xo(1:3);
end
figure
subplot(2,4,1)
plot(t_data,Fcmd_data(:,1),t_data,Fcmd_data(:,2),t_data,Fcmd_data(:,3));
axis([0 5 -.5 .5])
legend('$F_x$','$F_y$','$F_z$','Interpreter','latex')
title('robot 1 command force')
subplot(2,4,2)
plot(t_data,Fcmd_data(:,4),t_data,Fcmd_data(:,5),t_data,Fcmd_data(:,6));
axis([0 5 -.5 .5])
legend('$F_x$','$F_y$','$F_z$','Interpreter','latex')
title('robot 2 command force')
subplot(2,4,3)
plot(t_data,tau1_data(:,1),t_data,tau1_data(:,2));
axis([0 5 -.5 .5])
legend('$\tau_1$','$\tau_2$','Interpreter','latex')
title('robot 1 torque')
subplot(2,4,4)
plot(t_data,tau2_data(:,1),t_data,tau2_data(:,2));
axis([0 5 -.5 .5])
legend('$\tau_1$','$\tau_2$','Interpreter','latex')
title('robot 2 torque')
subplot(2,4,5)
plot(t_data,Xo_err(:,1),t_data,Xo_err(:,2),t_data,Xo_err(:,6));
axis([0 5 -.5 .5])
legend('$x$','$y$','$\theta$','Interpreter','latex')
title('object position error')
subplot(2,4,6)
plot(t_data,x1_err(:,1),t_data,x1_err(:,2));
axis([0 5 -.5 .5])
legend('$x$','$y$','Interpreter','latex')
title('robot 1 end-effector error')
subplot(2,4,7)
plot(t_data,x2_err(:,1),t_data,x2_err(:,2));
axis([0 5 -.5 .5])
legend('$x$','$y$','Interpreter','latex')
title('robot 2 end-effector error')

