clear all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Objective: Adaptive Cruise Control (ACC) Simulation (A repeat of the result
%of the paper "Control Barrier Function Based Quadratic Programs for Safety
%Critical Systems (Page 3870)")
%Author: Ming Li
%Date: March 2nd. 2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global u_save
%% Initial Setup
% Note that [x(1) x(2) x(3)]==[v_f v_l D]
%Initial distance between two vehicles is 150
%Intial velocity of the leading vehicle is 18;
%Intial velocity of the leading vehicle is 10;
Initial_position=[18 10 150];
t_span=[0:0.1:100];                                                        % Running time and interval

[Init_Par]=Initial_Parameter();
% [x] = ode4(@odefcn,t_span,Initial_position);
[x] = ode4(@odefcn,t_span,Initial_position);
% %% Ode45 plot
% figure(1)
% plot(x.y(1,:),'b--','linewidth',2)
% hold on
% plot(x.y(2,:),'r--','linewidth',2)
% set(gca,'FontSize',23)
% set(gcf,'Position',[200,200,1000,800], 'color','w')
% xlabel('x1')
% ylabel('x2')
% legend('$v_{f}$','$v_{l}$','Interpreter','latex')
% grid on

%% Ode4 plot
figure(1)
plot(t_span,x(:,1),'b--','linewidth',2)
hold on
plot(t_span,x(:,2),'r--','linewidth',2)
set(gca,'FontSize',23)
set(gcf,'Position',[200,200,1000,800], 'color','w')
xlabel('x1')
ylabel('x2')
legend('$v_{f}$','$v_{l}$','Interpreter','latex')
grid on

figure(2)
plot(x(:,3),'b--','linewidth',2)
set(gca,'FontSize',23)
set(gcf,'Position',[200,200,1000,800], 'color','w')
xlabel('x')
ylabel('D')
legend('$D$','Interpreter','latex')
grid on

%% Compute the CBF
h_safe=x(:,3)-Init_Par.tau_d*x(:,1);
figure(3)
plot(t_span,h_safe,'b--','linewidth',2)
set(gca,'FontSize',23)
set(gcf,'Position',[200,200,1000,800], 'color','w')
xlabel('x')
ylabel('$h_{S}$','Interpreter','latex')
legend('$h_{S}$','Interpreter','latex')
grid on

%% Compute the control input
% d_x=diff(x(:,1));
d_x=(x(2:end,1)-x(1:end-1,1))*10;
for i_u=1:size(x,1)-1
F_r(i_u)=Init_Par.f_0+Init_Par.f_1*x(1)+Init_Par.f_2*(x(1))^2;% Rolling resistance (force)
u_nodevi(i_u)=(d_x(i_u)*Init_Par.M+F_r(i_u));
u(i_u)=(d_x(i_u)*Init_Par.M+F_r(i_u))/Init_Par.M/Init_Par.a_g;
end
figure(4)
plot(t_span(2:end),u,'b-','linewidth',2)
set(gca,'FontSize',23)
set(gcf,'Position',[200,200,1000,800], 'color','w')
xlabel('t')
ylabel('$u$','Interpreter','latex')
legend('$u$','Interpreter','latex')
grid on

figure(5)
for i_u=1:size(x,1)-1
F_r(i_u)=Init_Par.f_0+Init_Par.f_1*x(1)+Init_Par.f_2*(x(1))^2;% Rolling resistance (force)
u_origin(i_u)=(u_save(1+(i_u-1)*4))/Init_Par.M/Init_Par.a_g;
end
plot(t_span(2:end),u_origin,'b-','linewidth',2)
set(gca,'FontSize',23)
set(gcf,'Position',[200,200,1000,800], 'color','w')
xlabel('t')
ylabel('$u$','Interpreter','latex')
legend('$u$','Interpreter','latex')
grid on



