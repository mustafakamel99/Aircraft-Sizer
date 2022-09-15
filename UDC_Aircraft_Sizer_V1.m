clc
close all
clearvars

%% Propulsion Design Parameters
% s_M = 6;                  % selected Battery cells from 18 to 27
% bat_cap = 18000;          % Selected Battery Capacity  
% thr_set = 70;             % Selected Throttle Setting 

%% battery weight calculation
%%[bat_w,bat_v] = Battery_weight(s_M,bat_cap);

g=9.81;
etta_p=0.55;
E_D=242*3600;
R=30*1000*2;
Cl_CDmax=6;
Cl_CD=0.8*Cl_CDmax;
WB_MTOW=1.05*((g*R)/(etta_p*E_D*Cl_CD))

%% UDC AIRFRAME SIZER
% Inputs # 1
box.n=1;                       %Number of vaccine vial packages
box.weight=1.5;              %weight of one vaccine vial package at kg
box.h=0.15;                %height of vaccine vial packages
box.w=0.15;                %width of vaccine vial packages
box.l=0.105;                 %length of vaccine vial packages
l=input('Please enter fuselage length :  ');
w=input('Please enter fuselage width  :   ');
h=input('Please enter fuselage height :   ');


extra_load=0.3;                   %weight of deploying mechanism
%mthd = 1;                     % Manufacturing technique [1 = Balsa , 2 = Foam Core Comp. , 3 = Hotwire Foam , 4 = Molded Comp.]
v_stall = 11;               % Min stall velocity 
%vstall_max = 11;              % Max stall velocity
cL_max_desired = 1.6;         % Desired cL max
V_c_min = 25;              % Min cruise velocity
V_c_max = 35;              % Max cruise velocity 
S_TO_min = 5;                % Min Take-off distance (m)
S_TO_max = 8;                % Max Take-off distance (m)
%% payload)
[pylod] = payload_weight (box,extra_load);
[fueslage]=fueslage_sizing(l,w,h);

%% Weight estimation
 %%[MTOW] = weight_guess(pylod,bat_w,mthd);
MTOW=6.5;
%[MTOW] = weight_guess_2(pylod,WB_MTOW,mthd)
W_B=MTOW*WB_MTOW
%% Constrains 
[v_cr,S] = constraints(v_stall,cL_max_desired,MTOW,V_c_max,V_c_min);
disp(S)

%% Inputs # 2

% wing.s = input('Please Select Desired Wing Area from the above range :   ');
wing.s =S
wing.AR = 6;                 % Wing aspect ratio (AR)
wing.TR = 1;                 % Wing taper ratio
wing.sweep = 0;              % Wing sweep angle
wing.Root_twist = 0;         % Wing root twist angle
wing.Tip_twist = 0;          % Wing tip twist angle

criteria = 1;                % Airfoil Selection Criteria [1 = CLmax , 2 = CDmin , 3 = L/D]
clmax_min_3D = 1.2;            % Minimum Acceptable CLmax
AOA_c = [0,4];               % Desired Trim AOA Range [min,max]
d_spar = 12;                 % Spar Diameter (mm)
                            
Sv_Sw = 0.12;                 % Horizontal Tail Area to Wing Area Ratio
htail.V_h = 0.5;            % Horizontal tail volume coefficient (VH)
vtail.V_v = 0.05;            % Vertical tail volume coefficient (VV)
htail.AR_h = 4;              % Horizontal tail aspect ratio
vtail.AR_v = 2;              % Vertical tail aspect ratio
SM = 10;                     % Static Margin percentage
% n = 5;                       % Load Factor
%% Wing_sizer
[wing.b,wing.Cr,wing.Ct,wing.MAC,wing.O_tip] = Wing_Sizer(wing);

if wing.b > 2.5
    span_string = num2str(wing.b);
    error("Span = " + " " + span_string + " m " + "..." + " You have exceeded the 1.5m Max Span Constraint, Reduce your ASPECT RATIO :D")   
end

%% Airfoil_Selector
[wing.Airfoil,CLmax,Tc_w] = Airfoil_Selector(MTOW,wing.s,wing.MAC,wing.AR,criteria,clmax_min_3D,AOA_c,d_spar,v_cr);

%% HLD_Sizer
[flaps,flap,CL_Max_flappped] = HLD_Sizer(CLmax,cL_max_desired,wing);
%% Tail_Sizer
 [htail.L,htail.S_h,htail.b_h,htail.C_h,vtail.S_v,vtail.b_v,vtail.C_v,tail.area,tail.DA,tail.bv,tail.cv] = Tail_Sizer(Sv_Sw,wing,htail,vtail);

% %% Stability_Trim
[X_cg,i_t,Alpha_trim_M,Cm_a_M,X_np ] = Stability_Trim(MTOW,wing,htail,vtail,tail,v_cr,SM);
% 
% %% Drag_polar
[v,Vc,D2_kg,CD0,C1,C2] = Drag_polar(wing,MTOW,tail,fueslage,Tc_w,V_c_min,V_c_max);
% %% Wing_Distribution
%  [Yle,ccl,cl] = Wing_Distribution(wing);
% % 
% % %% Power Calculatiron
 [Min_P,Max_P,Min_S_T,Min_D_T] = Constraints_2(wing,MTOW,CD0,S_TO_min,S_TO_max,CL_Max_flappped,V_c_min,V_c_max); 
 %,M3.MTOW_3,CD0_2,CD0_3,S_TO_min,S_TO_max,CL_Max_flappped,V_c_min,V_c_max,V_c_M3_min,V_c_M3_max);
% % 
% % %% Airfoil Plotter
%[dataX,dataY] = airfoil_plotter(wing);
% 
% % %% Figures
 % Figures(wing,htail,tail,dataX,dataY,Alpha_trim_M,Cm_a_M,Yle,ccl,cl,v,D2_kg,X_cg)
% 
% %% Displaying Most Important Results
 %Important_Data(wing,flap,tail,CLmax,MTOW,pylod,bat_w,CD0,Vc)
% 
% % End of AIRFRAME SIZER
% 
% %% Propulsion sizer function
% 
% [M2.C1_M2_Th,M2.C2_M2_Th,M3.C1_M3_Th,M3.C2_M3_Th,M2.Ts_100,M3.Ts_100,M3.Ts_any,M2.I_max,...
%     M3.I_max,M3.I_throttle,M3.C1_M3_100,M3.C2_M3_100] = Propulsion_Sizer_Main(V_c_min,...
%     V_c_M3_min,Min_S_T_M2,Min_S_T_M3,Min_D_T_M2,Min_D_T_M3,M2.C1,M2.C2,M3.C1,M3.C2,Min_P,...
%     Max_P,bat_v,bat_v_M3,M2.bat_cap,M3.bat_cap,thr_set,fueslage.w,fueslage.h);
% 
% 
% %% Mission Model
% 
% [Time_for_M2,distance_takeoff_M2,Time_for_M3,distance_takeoff_M3,no_of_laps_M3,Score_M2,Score_M3,Overall_score] = Mission_model_v1(wing,...
%  CL_Max_flappped,n,M2,MTOW_2,M3,score,syringes);

      