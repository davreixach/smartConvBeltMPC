%% MPC - Case Study
% David Reixach Pérez - 11.06.18

%% Initialization
clearvars -except f1 f2 f3 f4, clc

%% 1. Study on the perturbation propagation

% Propagated state variance (N steps) from a stabilized system with stable
% initial conditions

% W = 0.25; % Perturbation variance
W = (0.1/3)^2;
A = 0.5; % Closed sytem A matrix

N = 20;

X = 0;
for i = 0:N-1
    X = X + A^(2*i)*W;
end

X

%% 2. Model preparation

syms T vk thetak
A = [1 0 -T*vk*sin(thetak);...
     0 1 T*vk*cos(thetak);...
     0 0 1]

B = [T*cos(thetak) 0;...
     T*sin(thetak) 0;...
     0 T]


%% reference +
% circular + line

Ts = 0.05;

R = 20;
gamma = linspace(0,pi/2,R/Ts);

ref(1,:) = [R*(1-cos(gamma)), R+Ts:Ts:2*R];
ref(2,:) = [R*sin(gamma), R*ones(1,R/Ts)];
ref(3,:) = [pi/2-gamma, zeros(1,R/Ts)];

% figure2
% plot(ref(1,:),ref(2,:),'*'), hold on
quivset = 1:40:800;
% quiver(ref(1,quivset),ref(2,quivset),cos(ref(3,quivset)),sin(ref(3,quivset)))

%% Drawing

% % Box
% sx = 7; sy = sx/sqrt(2);
% [bx,by] = meshgrid(-sx/2:0.1:sx/2,-sy/2:0.1:sy/2);
% 
% box = [bx(:,end) by(:,end);bx(1,:)' by(1,:)';bx(:,1) by(:,1);bx(end,:)' by(end,:)'];
% 
% idbox = 250;
% tf = trotz(ref(3,idbox)*180/pi);
% boxt = zeros(size(box));
% for i=1:length(box)
%     boxt3 = tf*transl([box(i,1:2),0]);
%     boxt(i,:) = (boxt3(1:2,4) + [ref(1,idbox);ref(2,idbox)])';
% end
% 
% % rollers
% [rollx,rolly] = meshgrid(-7.5:2.5:37.5,2.5:2.5:22.5);
% roll = [rollx(:), rolly(:)];
% 
% % % figure2
% % plot(ref(1,:)+10,ref(2,:),'.','MarkerEdgeColor','k')
% % hold on
% % plot(boxt(:,1)+10,boxt(:,2),'.','MarkerEdgeColor','k')
% % plot(roll(:,1)+10,roll(:,2),'.','MarkerEdgeColor','k')
% % axis([0 50 0 25])
% % xlabel('x')
% % ylabel('y')

%% Optimizer
% yalmip('clear')
% 
% Am = sdpvar(3,3);
% Bm = sdpvar(3,2);
% u0 = sdpvar(2,1);
% E = [1;1;0.1]; 
% 
% 
% % Factors
% Q = eye(3)*1000;                     % penalty on states
% % Q(3,3) = 1e6;
% R = [0.1 0; 0 0.001];                % penalty on control action
% N = 5;                               % Horizon
% bu = [10;4];                         % -bu <= u <= bu
% 
% 
% it = length(ref)-N;                 % number of iterations
% nx = 3;                             % number of states
% nu = 2;                             % number of inputs
% 
% % Initial condition
% xk = zeros(3,it);
% xk(:,1) = ref(:,1);                 % initial condition for states
% dxk(:,1) = zeros(3,1);              % deviation from equilibrium point
% uk = zeros(2,it);
% 
% % Variable declaration
% u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
% x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% r = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% d = sdpvar(1);
% 
% constraints = [];
% objective = 0;
% for k = 1:N
%     objective = objective + (r{k}-x{k})'*Q*(r{k}-x{k}) + u{k}'*R*u{k};
%     constraints = [constraints, x{k+1} == Am*x{k}+Bm*u{k}];
%     constraints = [constraints, -bu <= u{k} + u0 <= bu];
% end
% objective = objective + (r{N+1}-x{N+1})'*Q*(r{N+1}-x{N+1});
% 
% parameters_in = {x{1},[r{:}],Am,Bm,u0};
% solutions_out = {[u{:}], [x{:}]};
% 
% controller = optimizer(constraints, objective,sdpsettings('solver','cplex'),parameters_in,solutions_out);
% 
% Ak = double(subs(A,[T,vk,thetak],[Ts,uk(1,1),xk(3,1)])); % initialization of linear model
% Bk = double(subs(B,[T,thetak],[Ts,xk(3,1)]));
% disturbance = normrnd(0,sqrt(W),1,it);
% % disturbance = zeros(1,it);
% for i = 1:it    
%     future_r = ref(:,i:i+N)-xk(:,i);    
%     inputs = {zeros(3,1),future_r,Ak,Bk,uk(:,max(i-1,1))};
%     [solutions,diagnostics] = controller{inputs};    
%     U = solutions{1};
%     X = solutions{2};
%     if diagnostics == 1
%         error('The problem is infeasible');
%     end
%     uk(:,i) = U(:,1) + uk(:,max(i-1,1));
% %     Ak = double(subs(A,[T,vk,thetak],[Ts,uk(1,i),xk(3,i)]));
% %     Bk = double(subs(B,[T,thetak],[Ts,xk(3,i)]));
%     
% %     xk(:,i+1) = Ak*zeros(3,1) + Bk*U(:,1) + xk(:,i) + E*disturbance(i);
%     xk(:,i+1) = nlmodel(xk(:,i),uk(:,i),Ts) + E*disturbance(i);
%     Ak = double(subs(A,[T,vk,thetak],[Ts,uk(1,i),xk(3,i)]));
%     Bk = double(subs(B,[T,thetak],[Ts,xk(3,i)]));
% 
% end

%% Non-linear simulation
% clear

nref = length(ref);
REF = [(0:Ts:(nref-1)*Ts)',ref',zeros(nref,2)];

% Factors
Q = eye(3)*100;                     % penalty on states
Q(3,3) = 80;
R = [1 0; 0 1];                % penalty on control action
Hp = 5;                              % Horizon
Hs = nref;
bu = [10;4];                         % -bu <= u <= bu


% Run
% robot_sp;
out = robot_sp_RUN();

% Rename
xk = out.STATES(:,2:end)';
uk = out.CONTROLS(:,2:end)';

quivset2 = floor(quivset*length(xk)/length(ref));

%% Plot

ts = 1:length(xk);
REFTS = (1:length(ref))*Ts;
% TS = ts*Ts; % For LMPC
TS = out.STATES(:,1);   %For NLMPC
% States-Ref Plot
% f1 = figure2
set(0,'CurrentFigure',f1);
subplot(311), plot(REFTS,ref(1,:),TS,xk(1,ts),'LineWidth',1)
xlabel('time [s]'), ylabel('x [u]'), legend('x_{r}','x','Location','southeast')
subplot(312), plot(REFTS,ref(2,:),TS,xk(2,ts),'LineWidth',1)
xlabel('time [s]'), ylabel('y [u]'), legend('y_{r}','y','Location','southeast')
subplot(313),pl = plot(REFTS,ref(3,:),TS,xk(3,ts),'LineWidth',1);
xlabel('time [s]'), ylabel('\theta [rad]'), legend('\theta{}_{r}','\theta','Location','northeast')
c = get(pl,'Color');

ts = 1:length(uk);
% TS = ts*Ts; % For LMPC
TS = out.CONTROLS(:,1);   %For NLMPC
% States-Ref Plot
% f2 = figure2;
set(0,'CurrentFigure',f2);
subplot(211), plot(TS,uk(1,:),'LineWidth',1,'Color',c{2})
xlabel('time [s]'), ylabel('v [u/s]')
subplot(212), plot(TS,uk(2,:),'LineWidth',1,'Color',c{2})
xlabel('time [s]'), ylabel('\omega [rad/s]')

% % X-Y Plot
% f3 = figure2;
set(0,'CurrentFigure',f3);
quiv1 = quiver(xk(1,quivset2)+10,xk(2,quivset2),cos(xk(3,quivset2)),sin(xk(3,quivset2)));
set(quiv1,'MaxHeadSize',0.3,'AutoScaleFactor',0.7,'Color','k'), hold on
f32 = scatter(ref(1,:)+10,ref(2,:),'.','MarkerEdgeColor',c{1});
f33 = scatter(xk(1,:)+10,xk(2,:),'.','MarkerEdgeColor',c{2}); hold off
xlabel('x [u]'), ylabel('y [u]'), legend([f32,f33],'Reference','Response','Location','southeast')

% COV = cov((xk-ref(:,1:it+1))')

fprintf('Control effort for v: %.2f\n',norm(uk(1,ts),2))
fprintf('Control effort for w: %.2f\n',norm(uk(2,ts),2))
ts = 1:length(xk);
fprintf('Tracking error for x1: %.2f\n',norm(xk(1,ts)-interp1(0:799,ref(1,:),out.STATES(:,1)/Ts)',2))
fprintf('Control effort for x2: %.2f\n',norm(xk(2,ts)-interp1(0:799,ref(2,:),out.STATES(:,1)/Ts)',2))
fprintf('Control effort for x3: %.2f\n\n',norm(xk(3,ts)-interp1(0:799,ref(3,:),out.STATES(:,1)/Ts)',2))

%% Plot surface

% % Position
% idx = randsample(length(xk),1)
% [~,idu] = min(abs(out.CONTROLS(:,1)-out.STATES(idx,1)));
% dist = (ref(1:2,:)'-out.STATES(idx,2:3));
% [~,idbox] = min(sum(dist.^2,2));
% 
% % Box
% tf = trotz(ref(3,idbox)*180/pi);
% boxt = zeros(size(box));
% for i=1:length(box)
%     boxt3 = tf*transl([box(i,1:2),0]);
%     boxt(i,:) = (boxt3(1:2,4) + [ref(1,idbox);ref(2,idbox)])';
% end
% 
% % Inverse Kinematics
% P = xk(1:2,idx)';
% Vp = [uk(1,idu)*cos(xk(3,idx)),uk(1,idu)*sin(xk(3,idx))];
% Omegap = uk(2,idu);
% X = roll;
% 
% V = Vp + [P(:,2)-X(:,2), -P(:,1)+X(:,1)]*Omegap;
% 
% % f4 = figure2;
% set(0,'CurrentFigure',f4);
% quiv1 = quiver(roll(:,1)+10,roll(:,2),V(:,1),V(:,2));
% set(quiv1,'MaxHeadSize',0.3,'AutoScaleFactor',0.7,'Color','k')
% hold on
% plot(ref(1,:)+10,ref(2,:),'.','MarkerEdgeColor','k')
% plot(boxt(:,1)+10,boxt(:,2),'.','MarkerEdgeColor','k')
% axis([0 50 0 25])
% xlabel('x')
% ylabel('y')
% hold off

%% Auxiliary Functions

function [x] = nlmodel(x,u,T)
%NLMODEL(x,u,T)
    x = x + [u(1)*T*cos(x(3));...
             u(1)*T*sin(x(3));...
             T*u(2)];
end

function [V] = ikine(P,Vp,Omegap,X)
%IKINEMATICS(P,Vp,Omegap,X)
V = Vp + [P(:,2)-X(:,2), -P(:,1)+X(:,1)]*Omegap;
end
