function   [thetak, disk] = motionsolve(KP2,KD2)
% for regular waves
% Naming principle：o - original，without control
% k - controlled
Ts = 0.1; % Sampling time
tj = 0:Ts:60;
T = length(tj);
m = 159.9;% mass
%_________________					
a33 =159.2604;
b33 =859.781;
a53 = 12.4722;
b53 =	189.8743253622;

a55 = 70.5159;
b55 = 228.8895976968;
a35 = 16.3098	;
b35 = 168.199;
I5 = 89.94375;% 纵向惯性矩
c33 = 12238;
c35 = 0;
c53 = 0;
c55 = 5748;
%% 定义波浪力、波浪力矩
Xw3a = 143.043863;
Xw5a = 60.1552629;
yiposilongX3 = 0.314159; 
yipoxilongX5 = 4.5727587778;
Xw3 = Xw3a*cos(5.97*tj+yiposilongX3);
Xw5 = Xw5a*cos(5.97*tj+yipoxilongX5);
Wt = [Xw3;Xw5];
%% Continuous-time state space model
%{
M*X_dot(t) = A*X(t)+C*w(t)+Bu(t)
u(t) = -K*X
%}
U = 1.6266;
kF = 4.859;
lF = 1.3; % T型翼作用中心至船中距离
M = [1 0 0 0;0 a33+m 0 a35;0 0 1 0;0 a53 0 I5+a55];
Ak = [0 1 0 0;-c33 -b33 -(c35-kF) -(b35+kF*(lF-1)/U);0 0 0 1;-c53 -b53 -(c55-kF*lF) -(b55+kF*lF*(lF-1)/U)] ;
B = -[0 kF 0 lF*kF]';
C = [0 0;1 0;0 0;0 1];
%{
X__dot(t) = A_*X(t)+B_*w(t)+C_W(t)
u(t) = -K*X
%}
A_ = inv(M) * Ak;
B_ = inv(M) * B;
C_ = inv(M) * C;
D = eye(4);
E = zeros(4,2);
%% 
%% 求解状态方程
% 定义PID gains
% KP1 = 0;
% KD1 = 0;
% KP2 = kp2;
% KD2 = kd2;
Ko = [0,0,0,0] ;%未控制
Kk = [0, 0, KP2,KD2];    %控制后
syso = ss((A_-B_*Ko),C_,D,E);
[~,~,Xo] = lsim(syso,Wt,tj); 

sysk = ss((A_-B_*Kk),C_,D,E);
[~,~,Xk] = lsim(sysk,Wt,tj); 
%%
x1o = Xo(:,1);
diso = max(x1o(551:601))
x3o = Xo(:,3);
thetao = max(x3o(551:601))
Ao = -5.37*5.37*x1o-5.37*5.37*1.5*x3o;% 艏加速度
ao = max(Ao(551:601));

x1k = Xk(:,1);
disk = max(x1k(551:601))
x3k = Xk(:,3);
thetak = max(x3k(551:601))
Ak = -5.37*5.37*x1k-5.37*5.37*1.5*x3k;%艏加速度
ak = max(Ak(551:601));
%% plot
% figure1 = figure('PaperType','<custom>','PaperSize',[16 9],'Color',[1 1 1]);
% axes1 = axes('Parent',figure1,...
%     'Position',[0.122088607594937 0.101822429906542 0.775 0.815]);
% hold(axes1,'on');
% plot1=plot(tj(300:600), x1k(300:600),tj(300:600),x1o(300:600));
% hold on
% set(plot1(1),'DisplayName','with active ride control','LineWidth',1.5,...
%     'Color',[1 0 0]);
% set(plot1(2),'DisplayName','with no ride control fitted','LineStyle','--',...
%     'Color',[0 0 1]);
% axis([30,60,-0.04,0.04])
% title('heave(Fn=0.3,λ/L=1.5)','FontName','Times New Roman','FontSize',20)
% xlabel('t/s','FontName','Times New Roman','FontSize',20)
% ylabel('z/m','FontName','Times New Roman','FontSize',20)
% box(axes1,'on');
% hold off
% set(axes1,'FontName','Times New Roman','FontSize',20);
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.680061020478919 0.79088785046729 0.21656345209492 0.124710865544377],...
%     'LineWidth',1,...
%     'FontSize',20);

% figure1 = figure('PaperType','<custom>','PaperSize',[16 9],'Color',[1 1 1]);
% axes1 = axes('Parent',figure1,...
%     'Position',[0.122088607594937 0.101822429906542 0.775 0.815]);
% hold(axes1,'on');
% plot1=plot(tj(300:600), x3k(300:600)*57.3,tj(300:600),x3o(300:600)*57.3);
% hold on
% set(plot1(1),'DisplayName','with active ride control','LineWidth',1.5,...
%     'Color',[1 0 0]);
% set(plot1(2),'DisplayName','with no ride control fitted','LineStyle','--',...
%     'Color',[0 0 1]);
% axis([30,60,-4.2,4.2])
% title('pitch(Fn=0.3,λ/L=1.5)','FontName','Times New Roman','FontSize',20)
% xlabel('t/s','FontName','Times New Roman','FontSize',20)
% ylabel('θ/°','FontName','Times New Roman','FontSize',20)
% box(axes1,'on');
% hold off
% set(axes1,'FontName','Times New Roman','FontSize',20);
% legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.680061020478919 0.79088785046729 0.21656345209492 0.124710865544377],...
%     'LineWidth',1,...
%     'FontSize',20);
% 

figure1 = figure('PaperType','<custom>','PaperSize',[16 9],'Color',[1 1 1]);
axes1 = axes('Parent',figure1,...
    'Position',[0.122088607594937 0.101822429906542 0.775 0.815]);
hold(axes1,'on');
plot1=plot(tj(300:600), Ak(300:600),tj(300:600),Ao(300:600));
hold on
set(plot1(1),'DisplayName','with active ride control','LineWidth',1.5,...
    'Color',[1 0 0]);
set(plot1(2),'DisplayName','with no ride control fitted','LineStyle','--',...
    'Color',[0 0 1]);
axis([30,60,-2.5,2.5])
title('bow-acceleration(Fn=0.3,λ/L=1.5)','FontName','Times New Roman','FontSize',20)
xlabel('t/s','FontName','Times New Roman','FontSize',20)
ylabel('α/g','FontName','Times New Roman','FontSize',20)
box(axes1,'on');
hold off
set(axes1,'FontName','Times New Roman','FontSize',20);
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.680061020478919 0.79088785046729 0.21656345209492 0.124710865544377],...
    'LineWidth',1,...
    'FontSize',20);
 end
