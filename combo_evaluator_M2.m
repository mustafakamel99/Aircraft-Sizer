function [Max_RPM_M2,I_max_100_M2,Ts_100,T_dyn_100,time_100,p_max_M2,Vmax,eta_M2,Pout,C1,C2]= combo_evaluator_M2...
    (GR,kv,kt,Rm,io,D,P,B,Etta_Th,bat_v_M2,bat_cap_M2,v_c_min_M2,P1,P2)

resc = 0.005; %% assuming
rbt = (9.9885*(bat_cap_M2)^-0.988)*(bat_v_M2/3.7);
R = rbt + resc + Rm;

kv = kv/GR;
kt = 9.5684/kv;
%%

nm = 1;
Pc = 1.08;
Tc = 1;

a = Pc * 4.0188e-15 * D.^4 .* P .*sqrt(B-1);
b = pi/30*kt*(1/(kv*R*nm));
c = - pi/30*kt*((bat_v_M2/(R*nm))-io);

Max_RPM_M2 = (-b+sqrt(b^2-4*a*c))./(2*a);

%% Calculating System Performance - Single Motor

% Propeller efficiency due to Blade Number

   eta_B = 1;  % for the propeller of two blades

%% Static Thrust ( 100% and Cruise Throttles )

Ts_100 = (eta_B * sqrt(B-1) * Tc * 2.6908e-9 .* D.^3 .* P .* Max_RPM_M2.^2).*Etta_Th;           

%% Dynamic Thrust ( In-Flight Thrust @ Vc 100% throttle )

Vp = P * 0.0254 .* Max_RPM_M2/60;    % Pitch Speed
C1 = -31/130 * Ts_100./(Vp.^2);      % Constant 1    
C2 = -0.4534 * Ts_100./Vp;           % Constant 2

T_dyn_100 = Ts_100 + C1*v_c_min_M2^2 + C2*v_c_min_M2; 

%% Current Draw

Ps = Pc * sqrt(B-1) * 4.0188e-15 .* D.^4 .* P .* Max_RPM_M2.^3;
Tau = Ps./(2*pi*Max_RPM_M2/60);
I_max_100_M2 = (Tau/kt + io);   % Amps

%% endurance_100
eta = 0.85;  % max discharge percentage 
time_100 = ((bat_cap_M2*60/1000)./I_max_100_M2)*eta;   % mins

%% max motor power in 
V_in = bat_v_M2-I_max_100_M2*(rbt+resc);
p_max_M2 = I_max_100_M2.*V_in;

%% Max Speed (Vmax)

A = [ P1-C1 , -C2 , -Ts_100 , zeros(length(C1),1) , ones(length(C1),1)*P2 ];
B = mat2cell(A,ones(1,size(A,1)),size(A,2));
C = cellfun(@roots,B,'UniformOutput',0);
D = cellfun(@transpose,C,'UniformOutput',0);
E = cell2mat(D);
Vmax = E(:,2);

%% System Efficiency @ Max Velocity

Thrust = Ts_100 + C1.*Vmax.^2 + C2.*Vmax; 
Pin = I_max_100_M2 * bat_v_M2;
Pout = Thrust*9.81*0.001.*Vmax;

eta_M2 = Pout./Pin;  % efficiency for M2
end