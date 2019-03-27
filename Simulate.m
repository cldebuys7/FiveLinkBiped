%%
% close all
p = param;
x = [q; dq];
polyOrder = 10;

tau = linspace(0,gait_time,25);
% polyp = polyfit(t,q1(1:nNomial)-q2(1:nNomial),polyOrder);
for i = 1:4
    k = 0;
    if i ==3
        k = pi;
    end

    polyp(i,:) = polyfit(tau,q(i+1,:)-q(i,:) + k,polyOrder);
    polyv(i,:) = polyp(i,1:polyOrder).*(polyOrder:-1:1);
    polya(i,:) = polyp(i,1:polyOrder-1).*(polyOrder-1:-1:1);

end
for i = 1:4
    yd(i,:) = polyval(polyp(i,:),tau);
    dyd(i,:) = polyval(polyv(i,:),tau);
    ddyd(i,:) = polyval(polya(i,:),tau);
end

polyq1d = polyfit(tau,q(1,:),polyOrder);
q1d = polyval(polyq1d,tau);

stepNum = 1;
samplingTime = 0.01;

% initial conditions
t0 = 0;
tspan2 = t0:samplingTime:2;
##z0 = [x(:,1); 0; 0; 0; 0];
z0 = [x(:,1)];

##figure(10001); clf;
##xlabel('angle')
##ylabel('angular velocity')
##hold on
##for i = 1: stepNum
    
    % define terminal event (impact definition)
    options=odeset('events',@(t,z) eventFun(t,z,p.l1,p.l2,p.l3,p.l4,p.l5));
    % dynamics
    tic
    [t,z] = ode45(@(t,z) calcDynamics(t,z,p,polyp,polyv,polya), tspan2, z0,options);%
    toc

##    t(end)
##    q1end = z(end,1);
##    q2end = z(end,2);
##    q3end = z(end,3);
##    q4end = z(end,4);
##    q5end = z(end,5);
##    dq1end = z(end,6);
##    dq2end = z(end,7);
##    dq3end = z(end,8);
##    dq4end = z(end,9);
##    dq5end = z(end,10);
##    q1p = z(1,1);
##    q2p = z(1,2);
##    q3p = z(1,3);
##    q4p = z(1,4);
##    q5p = z(1,5);
##    % impact dynamics and joint relabeling
##    [m,mi,f,fi,mz,mzi,mzd,fz,fzi,fzd] = autoGen_cst_heelStrike(q1p,q2p,q3p,q4p,q5p,q1end,q2end,q3end,q4end,q5end,dq1end,dq2end,dq3end,dq4end,dq5end,p.m1,p.m2,p.m3,p.m4,p.m5,p.I1,p.I2,p.I3,p.I4,p.I5,p.l1,p.l2,p.l3,p.l4,p.l5,p.c1,p.c2,p.c3,p.c4,p.c5,0);
##    M = zeros(5,5);  %Mass matrix
##    F = zeros(5,1);
##    M(mi) = m(:,1);
##    F(fi) = f(:,1);
##    dqpDyn = M\F;  %Numerically invert the mass matrix
##    
##    qNew = [q5end; q4end; q3end; q2end; q1end];
##    dqNew = dqpDyn;
##    z01 = [qNew;dqNew;z(end,11:14)'];
##    z01' - z0'
##    z0 = z01;
##% plot a phase portait for stance and swing    
##%     plot(z(:,1),z(:,6),'bo-')
##%     plot(z(:,5),z(:,10),'ro-')
####    plot(z(:,1),z(:,6),'bo-',z(:,5),z(:,10),'ro-')
##
##end

% The control input is integrated if it is included in the state
%   This takes the derivative to get the control input
##h = gait_time/size(z,1);
##u1 = gradient(z(:,11),h);
##u2 = gradient(z(:,12),h);
##u3 = gradient(z(:,13),h);
##u4 = gradient(z(:,14),h);
##uz = [u1 u2 u3 u4];

% this function calculates u if it is not in the state
u = calcInput(t,z(:,1:10),p,polyp,polyv,polya);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Anim.figNum = 1; clf(Anim.figNum);
##figure(1); clf;
##Anim.speed = 0.25;
qz = z(:,1:5)';
##Anim.plotFunc = @(t,qz)( drawRobot(qz,param) );
##Anim.verbose = true;
##animate(t,qz,Anim);

figure(6); clf;
subplot(1,2,1);
plot(t,[qz(1,:);qz(2,:)-qz(1,:);qz(3,:)-qz(2,:);qz(4,:)-qz(3,:)+pi;qz(5,:)-qz(4,:)]);
hold on
plot(tau,[q1d;yd],'--k')
legend('q1','q2','q3','q4','q5','q1d','q2d','q3d','q4d','q5d');
xlabel('time')
ylabel('link angles')
subplot(1,2,2);
plot(t,uz);
legend('u1','u2','u3','u4','u5');
xlabel('time')
ylabel('joint torques')