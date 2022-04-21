function [Init_Par]=Initial_Parameter()
% Note that the following parameters can be found in paper "Fixed-time Control
% under Spatiotemporal and Input Constraints: A Quadratic Program Based Approach"
% Or paper "Control Barrier Function based Quadratic Programs with Application to
% Adaptive Cruise Control" (In this paper, the following vehicle maintains a constant speed)
Init_Par.f_0=0.1;
Init_Par.f_1=5;
Init_Par.f_2=0.25;
Init_Par.M=1650;
Init_Par.a_l=0.3;                                                          %Acceleration of the leading vehicle
Init_Par.T_FxT=10;                                                         %Fix time for convergence
Init_Par.mu=5;                         
Init_Par.p=1+1/Init_Par.mu;
Init_Par.q=1-1/Init_Par.mu;
Init_Par.v_d=22;                                                       %Desired velocity of the following vehicle
Init_Par.tau_d=1.8;
Init_Par.p_sc=1e2;
Init_Par.c_convergence_rate=4;
Init_Par.gamma=0.5;

%% Bound of the control inout
Init_Par.a_g=9.81;
Init_Par.u_max=0.25*Init_Par.M*Init_Par.a_g;
end

