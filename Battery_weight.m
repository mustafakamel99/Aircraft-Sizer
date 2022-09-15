function [bat_w,bat_v] = Battery_weight(s_M,bat_cap)      
bat_v = s_M*3.7 ;       
      


bat_WH = bat_cap* bat_v/1000;   % calculating the capacity in WH 
energy_density = 140;                    % assuming range from 140 to 150
bat_w = bat_WH/energy_density;     % calculating Battery Weight for M2


