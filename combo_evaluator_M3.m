function [Max_RPM_M3,I_max_100_M3,Ts_100,T_dyn_M3,time_M3,p_max_M3,I_throttle,Vmax,eta_M3,Pout,C1_thr,C2_thr,Ts,C1_100,C2_100]= combo_evaluator_M3...
    (GR,kv,kt,Rm,io,D,P,B,Etta_Th,bat_v_M3,bat_cap_M3,v_c_min_M3,thr_set,P1,P2)

rbt = (9.9885*(bat_cap_M3)^-0.988)*(bat_v_M3/3.7);
resc = 0.005;       %assuming 
R = rbt + resc + Rm;

kv = kv/GR;
kt = 9.5684/kv;
%%
nm = 1;
Pc = 1.08;
Tc = 1;

a = Pc * 4.0188e-15 * D.^4 .* P .*sqrt(B-1);
b = pi/30*kt*(1/(kv*R*nm));
c = - pi/30*kt*((bat_v_M3/(R*nm))-io);

Max_RPM_M3 = (-b+sqrt(b^2-4*a*c))./(2*a);
RPM = Max_RPM_M3*thr_set/100;      %% @ at any throttle setting 

%% Calculating System Performance - Single Motor

% Propeller efficiency due to Blade Number

   eta_B = 1;     % for the propeller of two blades

%% Static Thrust ( 100% and Cruise Throttles )

Ts_100 = (eta_B * sqrt(B-1) * Tc * 2.6908e-9 .* D.^3 .* P .* Max_RPM_M3.^2).*Etta_Th; 
Ts = (eta_B * sqrt(B-1) * Tc * 2.6908e-9 .* D.^3 .* P .* RPM.^2).*Etta_Th;

%% Dynamic Thrust ( In-Flight Thrust @ x% throttle )

Vp = P * 0.0254 .* RPM/60;         % Pitch Speed
C1_thr = -31/130 * Ts./(Vp.^2);    % Constant 1    
C2_thr = -0.4534 * Ts./Vp;         % Constant 2

T_dyn_M3 = Ts + C1_thr*v_c_min_M3^2 + C2_thr*v_c_min_M3; % Dynamic Thrust @ any throttle

%% Dynamic Constants for 100% Throttle 

Vp_100 = P * 0.0254 .* Max_RPM_M3/60;      % Pitch Speed
C1_100 = -31/130 * Ts_100./(Vp_100.^2);    % Constant 1    
C2_100 = -0.4534 * Ts_100./Vp_100;         % Constant 2

% T_dyn_M3_100 = Ts_100 + C1_100*v_c_min_M3^2 + C2_100*v_c_min_M3;

%% Current Draw

Ps_100 = Pc * sqrt(B-1) * 4.0188e-15 .* D.^4 .* P .* Max_RPM_M3.^3;
Tau_100 = Ps_100./(2*pi*Max_RPM_M3/60);
I_max_100_M3 = (Tau_100/kt + io);

Ps = sqrt(B-1) * Pc * 4.0188e-15 .* D.^4 .* P .* (RPM).^3;   % Power @ any throttle
Tau = Ps./(2*pi*RPM/60);             % Torque @ any throttle 
I_throttle = (Tau/kt + io);          % Amps @ any throttle

%% endurance
eta = 0.85;  % max discharge percentage 
time_M3 = ((bat_cap_M3*60/1000)./I_throttle)*eta;  % mins

%% max motor power in 
V_in = bat_v_M3-I_max_100_M3*(rbt+resc);
p_max_M3 = I_max_100_M3.*V_in;

%% Max Speed (Vmax)

A = [ P1-C1_thr , -C2_thr , -Ts , zeros(length(C1_thr),1) , ones(length(C1_thr),1)*P2 ];
B = mat2cell(A,ones(1,size(A,1)),size(A,2));
C = cellfun(@roots,B,'UniformOutput',0);
D = cellfun(@transpose,C,'UniformOutput',0);
E = cell2mat(D);
Vmax = E(:,2);

%% System Efficiency @ Max Velocity at that Throttle Setting

Thrust = Ts + C1_thr.*Vmax.^2 + C2_thr.*Vmax; 
Pin = I_throttle * bat_v_M3;
Pout = Thrust*9.81*0.001.*Vmax;

eta_M3 = Pout./Pin;  % efficiency for M3
end 