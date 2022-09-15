function [v_cr,S] = constraints(v_stall,cL_max_desired,MTOW,V_c_max,V_c_min)
    v_cr=(V_c_max+V_c_min)/2;
    
rho = 1.225;
% v = linspace(vstall_min,vstall_max,5);

S = (2*MTOW*9.81)./(rho*v_stall.^2*cL_max_desired);
end

