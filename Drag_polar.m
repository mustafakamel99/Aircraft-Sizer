function[v,Vc,D2_kg,CD0,C1,C2] = Drag_polar(wing,MTOW,tail,fueslage,Tc_w,V_c_min,V_c_max)
Vc = (V_c_min+V_c_max)/2;
%INPUTS:
AR = wing.AR ;
S_ref = wing.s;
MAC = wing.MAC;
Cr = wing.Cr;
Ct = wing.Ct;
b = wing.b;
Xc =0.3;
% L_ht = htail.C_h;
% L_vt = vtail.C_v;
L_VT = tail.cv;

% Tc_ht = 0.09; %assume using NACA 0009
% Tc_vt = 0.09; %assume using NACA 0009
Tc_VT = 0.09; %assume using NACA 0009

% Sp_ht = htail.S_h;
% Sp_vt = vtail.S_v;
Sp_VT = tail.area;

Lf = fueslage.l;
Df = fueslage.w;

%assume fully turbulent flow 
%CD0 = CF*FF*Swet/Sref
%CF = 0.455/(log10(Re)^2.58)function of Reynolds number based on mean chord
%length
%SI units 
%calculate Re for each element for M2 
Re_w = 1.225*Vc*MAC/(1.802*10^-5);
Re_f = 1.225*Vc*Lf/(1.802*10^-5);
% Re2_ht = 1.225*Vc*L_ht/(1.802*10^-5);
% Re2_vt = 1.225*Vc*L_vt/(1.802*10^-5);
Re_VT = 1.225*Vc*L_VT/(1.802*10^-5);

%calculate Cf for each element for M2
CF_w = 0.455/(log10(Re_w)^2.58);
CF_f = 0.455/(log10(Re_f)^2.58);
% CF2_ht = 0.455/(log10(Re2_ht)^2.58);
% CF2_vt = 0.455/(log10(Re2_vt)^2.58);
CF_VT = 0.455/(log10(Re_VT)^2.58);
%calculate Cf for each element for M3
% CF3_w = 0.455/(log10(Re3_w)^2.58);
% CF3_f = 0.455/(log10(Re3_f)^2.58);
% CF3_ht = 0.455/(log10(Re3_ht)^2.58);
% CF3_vt = 0.455/(log10(Re3_vt)^2.58);
% CF3_VT = 0.455/(log10(Re3_VT)^2.58);
%calculate FF for each element
FF_w = 1+(0.6/Xc)*Tc_w/100 + 100*(Tc_w/100)^4;
% FF_ht = 1+(0.6/Xc)*Tc_ht + 100*(Tc_ht)^4;
% FF_vt = 1+(0.6/Xc)*Tc_vt + 100*(Tc_vt)^4;
FF_VT = 1+(0.6/Xc)*Tc_VT + 100*(Tc_VT)^4;
f_f = Lf/Df ;%refrence lenght/diamter of fuselag
FF_f = 1+(60/(f_f^3))+(f_f/400);
%calculate planform and wetted  area for each element 
Sp_w = 0.5*(Cr+Ct)*b ;
Sw_w = 2*(1+0.2*Tc_w/100)*Sp_w;
% Sw_ht = 2*(1+0.2*Tc_ht)*Sp_ht;
% Sw_vt = 2*(1+0.2*Tc_vt)*Sp_vt;
Sw_VT = 2*(1+0.2*Tc_VT)*Sp_VT;
Sw_f = 2*pi*Lf*Df/2;
%calculate CD0 for each element for M2
CD0_w = CF_w*FF_w*Sw_w/S_ref;
% CD02_ht = CF2_ht*FF_ht*Sw_ht/S_ref;
% CD02_vt = CF2_vt*FF_vt*Sw_vt/S_ref;
CD0_VT = CF_VT*FF_VT*Sw_VT/S_ref;
CD0_f = CF_f*FF_f*Sw_f/S_ref;
CD0 = (CD0_w+CD0_f+CD0_VT+0.0105)*1.15; % assume CD0 for landing gear = 0.0105


%calculate CDi
%CDi = cl^2/(e*pi*AR)
v = linspace (10,60,100);
%calculate drag polar for mission_2 (MTOW_2)
C1 = 0.5*1.225*CD0*S_ref ;
C2 = (2*(MTOW*9.81)^2) /(1.225*S_ref*pi*0.85*AR);
D_2 = C1 .*v.^2 + C2 ./(v.^2) ; %(N)
D2_kg = D_2 ./9.81 ;

end





