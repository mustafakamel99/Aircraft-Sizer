function [fuslage,container]=fuslage_sizing(container,sensor,sensor_n)
container.l = sensor.l+2*0.007;
container.d=sensor.d+2*0.007;
total_vol = container.l*container.d*container.d*sensor_n;
fuslage.w =  (((total_vol/4)^(1/3))) ; %assume AR for fuslage = 5 ,vol = 5w^3
if (rem(fuslage.w,container.d)~=0)
    fuslage.w = ceil(fuslage.w/container.d)*container.d;
end
fuslage.h= fuslage.w;
fuslage.l=  ((4*fuslage.w/0.6));      %assume fuslage usefl length = 0.6
if((fuslage.w/container.d)^2<sensor_n)
    fuslage.l = ceil(fuslage.l/container.l)*container.l/0.6;
end
