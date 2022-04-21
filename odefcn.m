function dxdt = odefcn(t,x)
global u_save
dxdt = zeros(3,1);
% Note that [x(1) x(2) x(3)]==[v_f v_l D]
% time
t
[Init_Par]=Initial_Parameter();
Init_Par.F_r=Init_Par.f_0+Init_Par.f_1*x(1)+Init_Par.f_2*(x(1))^2;% Rolling resistance (force)

%% Three different types of constraints
%% Hard Constraints
%These represent constraints that must not be violated under any condition.
%For ACC, this is simply the constraint: "keep a safe distance from the car
%infront of you". To guarantee the above specification, we use the condition:
%the minimum distance between two cars is "half the speedometer". This
%translate into the hard constraint: D>=tau_d*v (where tau_d is the desired
%time headway). h_safe=D-tau_d*v_f;
h_safe=x(3)-Init_Par.tau_d*x(1);
% if h_safe<0
%     x=x_save(:,end-1);
%     x_save(:,end)=[];
%     u_save(:,end)=[];
%     Init_Par.gamma=0;
%     h_safe=0;
% else
%     Init_Par.gamma=1;
% end


% n_par=2;
% if h_safe<0
%     Inner_state=x_save(:,end-1);
%     Outer_state=x_save(:,end);
%     cvx_begin quiet
%     variable lambda(n_par,1)
%     minimize(lambda.'*lambda)
%     subject to
%         (lambda(1)*Inner_state(3)+lambda(2)*Outer_state(3))-Init_Par.tau_d*(lambda(1)*Inner_state(1)+lambda(2)*Outer_state(1))<=1e-2;
%         (lambda(1)*Inner_state(3)+lambda(2)*Outer_state(3))-Init_Par.tau_d*(lambda(1)*Inner_state(1)+lambda(2)*Outer_state(1))>=0;
%         abs(lambda(1)+lambda(2)-1)<=1e-2;
%     cvx_end
%     x=lambda(1)*Inner_state+lambda(2)*Outer_state;
% end


%% Soft Constraints
% In the context of ACC, when adequate headway is assured, the goal is to
% achieve a desired speed, v_d. In other words, we expect
% lim_{t\rightarrow\infty}v_f(t)=v_d. This translates into a soft
% constraint can be written
% h_goal=(x(1)-Init_Par.v_d)^2;
h_goal=(x(1)-Init_Par.v_d)^2;

% %% Force Constraints
% % Note that the descriptions of this type of constraints can be found in paper
% % "Control Barrier Function based Quadratic Programs with Application to Adaptive Cruise Control"
% % These Constraints describe allowable wheel forces that are consistent with the driver convenience
% % aspect of ACC; these are typically much less than the peak forces that can be generated by the car
% % in emergency situations. Supposing that we do not want to accelerate or decelerate mote than some
% % fraction of g, we can write the constraints on acceleration and deceleration as an inequality:
% % -c_d*g<=F_w/m<=c_a*g; Basically, this inequality is equivalent to
% % -u_min<=u<=u_max in our formulation. Specifically, u_min=c_d*g*M, and we
% % deine c_d=0.25, g=9.81 and M=1650 in Initial_Parameter.m
% u>=-u_max;u<=u_max;

%% Formulate and Solve the QPs
%% Use function quadprog
Lf_goal=-2*(x(1)-Init_Par.v_d)*(Init_Par.F_r)/Init_Par.M;
Lg_goal=2*(x(1)-Init_Par.v_d)/Init_Par.M;
Lf_hS = x(2)-x(1)+Init_Par.tau_d*Init_Par.F_r/Init_Par.M;
Lg_hS = -Init_Par.tau_d/Init_Par.M;
A=[Lg_goal,-1;-Lg_hS,0];
b=[-Lf_goal-Init_Par.c_convergence_rate*h_goal,Lf_hS+Init_Par.gamma*h_safe].';
% A=[Lg_goal,-1;-Lg_hS,0];
% b=[-Lf_goal-Init_Par.c_convergence_rate*h_goal,Lf_hS].';
H=2*[1/(Init_Par.M^2),0;0,Init_Par.p_sc];
F=-2*[Init_Par.F_r/(Init_Par.M^2),0].';
% F=-2*[0,0].';
[u]=quadprog(H,F,A,b);
u_save=[u_save,u(1)];
% a_L=0;
%Store the generated control input
% %% Define the acceleration of the leading vehicle
if t<=8
    a_L = 0;
elseif t<=18
    a_L = 1.2;
elseif t<=30
    a_L = 0;
elseif t<=40
    a_L = 0.5;
elseif t<=50
    a_L = 0;
elseif t<=60
    a_L = -1;
else
    a_L = 0;
end

%% Dynamics of the system
dxdt(1)=-Init_Par.F_r/Init_Par.M+u(1)/Init_Par.M;                           %Note that u is essentially the wheel force
dxdt(2)=a_L;
dxdt(3)=x(2)-x(1);
end