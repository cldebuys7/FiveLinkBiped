% MAIN.m  --  Five Link Biped trajectory optimization
%
% This script sets up and then solves the optimal trajectory for the five
% link biped, assuming that the walking gait is compused of single-stance
% phases of motion connected by impulsive heel-strike (no double-stance or
% flight phases).
%
% The equations of motion and gradients are all derived by:
%   --> Derive_Equations.m 
%

clc; clear; 
addpath ../../

%>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%
%                       Define human-like trajectories                    %
%<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<%
data = load('kinematics3.mat');
data1 = data.kin(18:51,:);
data2 = data.kin(52:61,:);

data.kin = [data1; data2];
m = size(data.kin,1);   % number of data points

gait_time = 1.23*m/100; % calcualtes time for one gait cycle in sec

% Desired joint trajectories (pos, vel, and acc) ! shank angle is off !
% qd =      [   zeros(m,1),         zeros(m,1),         zeros(m,1), ...
%         90-data.kin(:,1),     -data.kin(:,2),  360-data.kin(:,3), ...
%        180+data.kin(:,4),  360+data.kin(:,5),  90+data.kin(:,6)]*pi/180;
% 
% qd_dot =  [   zeros(m,1),         zeros(m,1),         zeros(m,1), ...
%           -data.kin(:,7),     -data.kin(:,8),     -data.kin(:,9), ...
%           data.kin(:,10),     data.kin(:,11),     data.kin(:,12)]*pi/180;
% 
% qd_ddot = [   zeros(m,1),         zeros(m,1),         zeros(m,1), ...
%          -data.kin(:,13),    -data.kin(:,14),    -data.kin(:,15), ...
%           data.kin(:,16),     data.kin(:,17),     data.kin(:,18)]*pi/180;
% 
% x_0 = [qd; qd_dot];
tspan = linspace(0,gait_time,m)';  % Time vector
% 
% % Convert to existing code convention (th = theta) % make function and use
% % th = A*qd + b
% th = [qd(:,1), qd(:,1)+qd(:,2), qd(:,1)+qd(:,2)+qd(:,3), ...
%     qd(:,1)+qd(:,2)+qd(:,3)+qd(:,4) - pi, ...
%     qd(:,1)+qd(:,2)+qd(:,3)+qd(:,4)+qd(:,5) - pi];
% 
% th_dot = [qd_dot(:,1), qd_dot(:,1)+qd_dot(:,2), qd_dot(:,1)+qd_dot(:,2)+qd_dot(:,3), ...
%     qd_dot(:,1)+qd_dot(:,2)+qd_dot(:,3)+qd_dot(:,4), ...
%     qd_dot(:,1)+qd_dot(:,2)+qd_dot(:,3)+qd_dot(:,4)+qd_dot(:,5)];
% 
% th_ddot = [qd_ddot(:,1), qd_ddot(:,1)+qd_ddot(:,2), qd_ddot(:,1)+qd_ddot(:,2)+qd_ddot(:,3), ...
%     qd_ddot(:,1)+qd_ddot(:,2)+qd_ddot(:,3)+qd_ddot(:,4), ...
%     qd_ddot(:,1)+qd_ddot(:,2)+qd_ddot(:,3)+qd_ddot(:,4)+qd_ddot(:,5)];
% 
% % Polynomial fits for desired trajectories (th)
% order = 10;
% polp = zeros((size(qd,2) - 3), (order + 1));
% polv = zeros((size(qd,2) - 3), (order + 1));
% pola = zeros((size(qd,2) - 3), (order + 1));
% for ndx = 1:(size(qd,2) - 3)
% polp(ndx,:) = polyfit(tspan,     qd(:,ndx+3),order);
% polv(ndx,:) = polyfit(tspan, qd_dot(:,ndx+3),order);
% pola(ndx,:) = polyfit(tspan,qd_ddot(:,ndx+3),order);
% end

% % Polynomial fits for desired trajectories (qd)
% order = 10;
% polyp = zeros((size(qd,2) - 3), (order + 1));
% polyv = zeros((size(qd,2) - 3), (order + 1));
% polya = zeros((size(qd,2) - 3), (order + 1));
% for ndx = 1:(size(qd,2) - 3)
% polyp(ndx,:) = polyfit(tspan,     qd(:,ndx+3),order);
% polyv(ndx,:) = polyfit(tspan, qd_dot(:,ndx+3),order);
% polya(ndx,:) = polyfit(tspan,qd_ddot(:,ndx+3),order);
% end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up parameters and options                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
param = getPhysicalParameters();

param.stepLength = 0.5;
% param.stepTime = 0.7;
param.stepTime = tspan(end);

% % store polynomial fits in param struct
% param.traj.pos = polp;
% param.traj.vel = polv;
% param.traj.acc = pola;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics =  @(t,x,u)( dynamics(t,x,u,param) );

problem.func.pathObj = @(t,x,u)( obj_torqueSquared(x,u) );

problem.func.bndCst = @(t0,x0,tF,xF)( stepConstraint(x0,xF,param) );

problem.func.pathCst = @(t,x,u)( pathConstraint(x) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = param.stepTime;
problem.bounds.finalTime.upp = param.stepTime;

% State: (absolute reference frames)
%   1 = stance leg tibia angle
%   2 = stance leg femur angle
%   3 = torso angle
%   4 = swing leg femur angle
%   5 = swing leg tibia angle

qLow = (-pi/2)*ones(5,1);
qUpp = (pi/2)*ones(5,1);
dqLow = -10*ones(5,1);
dqUpp = 10*ones(5,1);
problem.bounds.state.low = [qLow; dqLow];
problem.bounds.state.upp = [qUpp; dqUpp];
problem.bounds.initialstate.low = [qLow; dqLow];
problem.bounds.initialstate.upp = [qUpp; dqUpp];
problem.bounds.finalstate.low = [qLow; dqLow];
problem.bounds.finalstate.upp = [qUpp; dqUpp];

uMax = 100;  %Nm
problem.bounds.control.low = -uMax*ones(5,1);
problem.bounds.control.upp = uMax*ones(5,1);

% Disable the stance ankle motor:
problem.bounds.control.low(1) = 0;
problem.bounds.control.upp(1) = 0;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values

problem.guess.time = [0, param.stepTime];

q0 = [...
    -0.3; % stance leg tibia angle
    0.7; % stance leg femur angle
    0.0; % torso angle
    -0.5; % swing leg femur angle
    -0.6]; % swing leg tibia angle
qF = q0([5;4;3;2;1]);   %Flip left-right

dq0 = (qF-q0)/param.stepTime;
dqF = dq0;

problem.guess.state = [q0, qF; dq0, dqF];

problem.guess.control = zeros(5,2);  %Start with passive trajectory


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


%NOTE:  Here I choose to run the optimization twice, mostly to demonstrate
%   functionality, although this can be important on harder problems. I've
%   explicitly written out many options below, but the solver will fill in
%   almost all defaults for you if they are ommitted.

method = 'trapezoid';
% method = 'trapGrad';   % This one is also good
% method = 'hermiteSimpson';
% method = 'hermiteSimpsonGrad';   % Suggested method
% method = 'chebyshev';   
% method = 'rungeKutta';  %slow!
% method = 'rungeKuttaGrad';
% method = 'gpops';

%%%% Method-independent options:
problem.options(1).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-3,...
    'MaxFunEvals',1e4);   %options for fmincon
problem.options(2).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-6,...
    'MaxFunEvals',5e4);   %options for fmincon

switch method
    
    case 'trapezoid'
        problem.options(1).method = 'trapezoid'; % Select the transcription method
        problem.options(1).trapezoid.nGrid = 10;  %method-specific options
        
        problem.options(2).method = 'trapezoid'; % Select the transcription method
        problem.options(2).trapezoid.nGrid = 25;  %method-specific options
        
    case 'trapGrad'  %trapezoid with analytic gradients
        
        problem.options(1).method = 'trapezoid'; % Select the transcription method
        problem.options(1).trapezoid.nGrid = 10;  %method-specific options
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
        problem.options(2).method = 'trapezoid'; % Select the transcription method
        problem.options(2).trapezoid.nGrid = 45;  %method-specific options
        problem.options(2).nlpOpt.GradConstr = 'on';
        problem.options(2).nlpOpt.GradObj = 'on';
        
    case 'hermiteSimpson'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = 6;  %method-specific options
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(2).hermiteSimpson.nSegment = 15;  %method-specific options
        
    case 'hermiteSimpsonGrad'  %hermite simpson with analytic gradients
        
        problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(1).hermiteSimpson.nSegment = 6;  %method-specific options
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
        problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
        problem.options(2).hermiteSimpson.nSegment = 15;  %method-specific options
        problem.options(2).nlpOpt.GradConstr = 'on';
        problem.options(2).nlpOpt.GradObj = 'on';
        
        
    case 'chebyshev'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'chebyshev'; % Select the transcription method
        problem.options(1).chebyshev.nColPts = 9;  %method-specific options
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).method = 'chebyshev'; % Select the transcription method
        problem.options(2).chebyshev.nColPts = 15;  %method-specific options
        
    case 'multiCheb'
        
        % First iteration: get a more reasonable guess
        problem.options(1).method = 'multiCheb'; % Select the transcription method
        problem.options(1).multiCheb.nColPts = 6;  %method-specific options
        problem.options(1).multiCheb.nSegment = 4;  %method-specific options
        
        % Second iteration: refine guess to get precise soln
        problem.options(2).method = 'multiCheb'; % Select the transcription method
        problem.options(2).multiCheb.nColPts = 9;  %method-specific options
        problem.options(2).multiCheb.nSegment = 4;  %method-specific options
        
    case 'rungeKutta'
        problem.options(1).method = 'rungeKutta'; % Select the transcription method
        problem.options(1).defaultAccuracy = 'low';
        problem.options(2).method = 'rungeKutta'; % Select the transcription method
        problem.options(2).defaultAccuracy = 'medium';
    
    case 'rungeKuttaGrad'
      
        problem.options(1).method = 'rungeKutta'; % Select the transcription method
        problem.options(1).defaultAccuracy = 'low';
        problem.options(1).nlpOpt.GradConstr = 'on';
        problem.options(1).nlpOpt.GradObj = 'on';
        problem.options(1).nlpOpt.DerivativeCheck = 'off';
        
        problem.options(2).method = 'rungeKutta'; % Select the transcription method
        problem.options(2).defaultAccuracy = 'medium';
        problem.options(2).nlpOpt.GradConstr = 'on';
        problem.options(2).nlpOpt.GradObj = 'on';
        
    case 'gpops'
        problem.options = [];
        problem.options.method = 'gpops';
        problem.options.defaultAccuracy = 'high';
        problem.options.gpops.nlp.solver = 'snopt';  %Set to 'ipopt' if you have GPOPS but not SNOPT
        
    otherwise
        error('Invalid method!');
end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%%% THE KEY LINE:
soln = optimTraj(problem);

% Transcription Grid points:
t = soln(end).grid.time;
q = soln(end).grid.state(1:5,:);
dq = soln(end).grid.state(6:10,:);
u = soln(end).grid.control;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Anim.figNum = 1; clf(Anim.figNum);
Anim.speed = 0.25;
Anim.plotFunc = @(t,q)( drawRobot(q,param) );
Anim.verbose = true;
animate(t,q,Anim);

figure(2); clf;
subplot(1,2,1);
plot(t,q);
legend('q1','q2','q3','q4','q5');
xlabel('time')
ylabel('link angles')
subplot(1,2,2);
plot(t,u);
legend('u1','u2','u3','u4','u5');
xlabel('time')
ylabel('joint torques')

if isfield(soln(1).info,'sparsityPattern')
   figure(3); clf;
   spy(soln(1).info.sparsityPattern.equalityConstraint);
   axis equal
   title('Sparsity pattern in equality constraints')
end





