function [Min_P,Max_P,Min_S_T,Min_D_T] = Constraints_2(wing,MTOW,CD0,S_TO_min,S_TO_max,CL_Max_flappped,V_c_min,V_c_max)
g=9.81;
e=0.85;
W_S=MTOW*g/wing.s;
% W_S_M3=MTOW_3*g/wing.s;
S_TO=[S_TO_min S_TO_max];
V_c=[V_c_min V_c_max];
% V_c_M_3=[V_c_M3_min V_c_M3_max];
sigma=1;
rho=1.225;
K=1/(pi*e*wing.AR);
n_p_TO=0.5;
n_p_c=0.7;
rho_0=1.225;
C_D_G=CD0+K*CL_Max_flappped^2;
% C_D_G_3=CD0_3+K*CL_Max_flappped^2;
V_stall_actual=sqrt((2*W_S)/(rho*CL_Max_flappped));
% V_stall_actual_3=sqrt((2*W_S_M3)/(rho*CL_Max_flappped));
V_TO=1.1*V_stall_actual;
% V_TO_3=1.1*V_stall_actual_3;
X_M=ones(1,2);
% X_M_3=ones(1,2);
W_P_S_TO=ones(1,2);
% W_P_S_TO_M3=ones(1,2);
W_P_V_max=ones(1,2);
% W_P_V_max_M3=ones(1,2);
for i=1:2
    X_M(i)=0.6*rho*g*C_D_G*S_TO(i)*(1/W_S);
%     X_M_3(i)=0.6*rho*g*C_D_G_3*S_TO(i)*(1/W_S_M3);
end
for p=1:2
    W_P_S_TO(p)=(n_p_TO*(1-exp(X_M(p))))/(V_TO*(0.05-((0.05+(C_D_G/CL_Max_flappped))*(exp(X_M(p))))));
    W_P_V_max(p)=(n_p_c)/((0.5*rho_0*V_c(p)^3*CD0*(1/(W_S)))+(((2*K)/(rho*sigma*V_c(p)))*(W_S)));
%     W_P_S_TO_M3(p)=(n_p_TO*(1-exp(X_M_3(p))))/(V_TO_3*(0.05-((0.05+(C_D_G_3/CL_Max_flappped))*(exp(X_M_3(p))))));
%     W_P_V_max_M3(p)=(n_p_c)/((0.5*rho_0*V_c_M_3(p)^3*CD0_3*(1/(W_S)))+(((2*K)/(rho*sigma*V_c_M_3(p)))*(W_S)));
end

W_P=[W_P_S_TO W_P_V_max];
% W_P=[W_P_S_TO_M3 W_P_V_max_M3];
P_M=(1./W_P).*MTOW.*g;
% P_M3=(1./W_P).*MTOW_3.*g;
P_S_TO=(1./W_P_S_TO).*MTOW.*g;
P_V_max=(1./W_P_V_max).*MTOW.*g;
% P_S_TO_M3=(1./W_P_S_TO_M3).*MTOW_3.*g;
% P_V_max_M3=(1./W_P_V_max_M3).*MTOW_3.*g;


Min_P_S_TO=min(P_S_TO);
% Min_P_S_TO_M3=min(P_S_TO_M3);
Min_P_V_max=min(P_V_max);
% Min_P_V_max_M3=min(P_V_max_M3);

Min_P_T=[Min_P_S_TO  Min_P_V_max ];
Min_P=max(Min_P_T);

Max_P=max(P_M);
% Max_P_M3=max(P_M3);

Min_S_T=(P_S_TO(2)*n_p_TO)/V_TO;
% Min_S_T_M3=(P_S_TO_M3(2)*n_p_TO)/V_TO_3;
Min_D_T=(P_V_max(1)*n_p_c)/V_c_min;
% Min_D_T_M3=(P_V_max_M3(1)*n_p_c)/V_c_M3_min;

% if Max_P_M3>Max_P
%     Max_P=Max_P_M3;
% else
%     Max_P=Max_P;
% end
end

