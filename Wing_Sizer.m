function[b,Cr,Ct,MAC,O_tip] = Wing_Sizer(wing)

area = wing.s; 
AR = wing.AR;
TR = wing.TR;
sweep = wing.sweep;

b = sqrt(AR*area);
Cr = 2*area/(b*(1+TR));
Ct = TR*Cr;
MAC = 2/3*Cr*((1+TR+TR^2)/(1+TR));
O_tip =(b*tand(sweep))/2- Ct/4 + Cr/4;



