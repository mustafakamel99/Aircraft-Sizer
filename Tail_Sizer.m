function [L,S_h,b_h,C_h,S_v,b_v,C_v,Vt,DA,b_vtail,c_vtail] = Tail_Sizer(Sv_Sw,wing,htail,vtail)

Sw = wing.s;
b = wing.b;
MAC = wing.MAC;
V_h = htail.V_h;
V_v = vtail.V_v;
AR_h = htail.AR_h;
AR_v = vtail.AR_v;

%procedures
L_v = V_v/Sv_Sw*b;
L_h = L_v;

S_v = Sv_Sw*Sw;
S_h = V_h*Sw*MAC/L_h;
Vt=S_h+S_v;
DA=atan(sqrt(S_v/S_h));

b_vtail = sqrt(AR_h*Vt);
c_vtail = b_vtail/AR_h;

b_h = sqrt(AR_h*S_h);
b_v = sqrt(AR_v*S_v);
C_h = b_h/AR_h;
C_v = b_v/AR_v;

L = L_h;
