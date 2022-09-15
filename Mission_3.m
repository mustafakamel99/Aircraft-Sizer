function [no_of_laps,Time_M3,distance_takeoff,cap_consumed_M3]= Mission_3(wing,cl_max,n,M3);

MTOW_3 = M3.MTOW_3;


Ts = M3.Ts_any;
Ts_100_M3 = M3.Ts_100 ;

I_max_100_M3 = M3.I_max ;
I_throttle = M3.I_throttle;

%note that c1,c2 are the coeifficients of Drag
c1 = M3.C1;
c2 = M3.C2;
%note that c3,c4 are the coeifficients of thrust-propulsion at any throttle
%note that c3_max,c4_max are the coeifficients of thrust-propulsion at max throttle
c3 = M3.P1_throttle;
c4 = M3.P2_throttle;

c3_max = M3.P1_max;
c4_max = M3.P2_max;

S_ref = wing.s;
AR = wing.AR;

 % Runge Katta approach
 h=.1;
 i=1;
 t9(i)=0;
 v9(i)=0;
 distance(i) = 0;
 c2_max = 0.5 *1.225*S_ref*cl_max^2/(pi*0.85*AR) ;
 L_takeoff = 0;

%CL turn & c2
CL_turn = 2*n*MTOW_3*9.81/(1.225*S_ref);
c2_turn = 0.5*1.225*S_ref*CL_turn^2/(pi*0.85*AR);

v_stall = sqrt((2*MTOW_3*9.806)/(1.225*S_ref*cl_max));
%%%%%%%%Lap 1%%%%%%%

%% takeoff
while (L_takeoff <= (MTOW_3*9.81))
     v_dot = @(t,v)((((Ts_100_M3-c3_max*v^2-c4_max*v)*0.001*9.81)-c1*v^2-c2_max*v^2-0.05*(0.5*1.225*v^2*S_ref*cl_max-(MTOW_3*9.81)))/MTOW_3);
     k1 = h*v_dot(t9(i),v9(i));
     k2 = h*v_dot(t9(i)+0.5*h,v9(i)+0.5*k1);
     k3 = h*v_dot(t9(i)+0.5*h,v9(i)+0.5*k2);
     k4 = h*v_dot(t9(i)+h,v9(i)+k3);
     t9(i+1) = t9(i)+h;
     v9(i+1) = v9(i)+1/6*(k1+2*k2+2*k3+k4);
     L_takeoff = 0.5*1.225*S_ref*(v9(i+1)^2)*cl_max; 
     distance(i+1) = distance(i)+(v9(i+1)+v9(i))*0.5*h;
     i=i+1;

 end
v_takeoff = v9(i);
t_takeoff = t9(i);
distance_takeoff = distance(i);

 %% climb
 i=1;
 altitude(i)=0;
 theta = 25;
 t10(i)=t_takeoff;
 v10(i)=v_takeoff;
 ROC(i)=0;
 while (altitude <= 20 )
     v_dot = @(t,v)((((Ts_100_M3-c3_max*v^2-c4_max*v)*0.001*9.81)-c1*v^2-c2_max*v^2-MTOW_3*9.81*sind(theta))/MTOW_3);
     k1 = h*v_dot(t10(i),v10(i));
     k2 = h*v_dot(t10(i)+0.5*h,v10(i)+0.5*k1);
     k3 = h*v_dot(t10(i)+0.5*h,v10(i)+0.5*k2);
     k4 = h*v_dot(t10(i)+h,v10(i)+k3);
     t10(i+1) = t10(i)+h;
     v10(i+1) = v10(i)+1/6*(k1+2*k2+2*k3+k4);
     ROC(i+1) = v10(i+1)*sind(theta);
     altitude(i+1) = altitude(i)+(ROC(i+1)+ROC(i))*0.5*h;
     i=i+1;
 end
v_climb =v10(i);
t_climb = t10(i);
altitude_climb = altitude(i);
distance_climb = altitude_climb/tand(theta);

time_takeoff_climb_hour = t_climb / 3600;
cap_consumed_M3_max_throttle = (I_max_100_M3 * 1000 * time_takeoff_climb_hour)/0.85;


%% cruise 0 (after climb)
i=1;
v11(i)=v_climb;
t11(i)=t_climb;
distance_cruise0(i)=0;
 while ( distance_cruise0 <( 150-distance_climb-distance_takeoff ))
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(t11(i),v11(i));
     k2 = h*v_dot(t11(i)+0.5*h,v11(i)+0.5*k1);
     k3 = h*v_dot(t11(i)+0.5*h,v11(i)+0.5*k2);
     k4 = h*v_dot(t11(i)+h,v11(i)+k3);
     t11(i+1) = t11(i)+h;
     v11(i+1) = v11(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise0(i+1) = distance_cruise0(i)+(v11(i+1)+v11(i))*0.5*h;
     i=i+1;
 end
 v_cruise0=v11(i);
t_cruise0=t11(i);
%% first turn (180 deg)
i=1;
high_load_factor=0;
v12(i) = v_cruise0;
t12(i) = t_cruise0;
Turn_angle7(i)=0;
 while Turn_angle7 < pi

     
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW_3);
     k1 = h*v_dot(t12(i),v12(i));
     k2 = h*v_dot(t12(i)+0.5*h,v12(i)+0.5*k1);
     k3 = h*v_dot(t12(i)+0.5*h,v12(i)+0.5*k2);
     k4 = h*v_dot(t12(i)+h,v12(i)+k3);
     v12(i+1) = v12(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if v12(i+1) < v_stall
         
         v12(i+1) = v_stall;
         high_load_factor=1;
     end
     
     angle_dot= @(t,Turn_angle7)(9.81*sqrt((n^2)-1)/v12(i+1));
     k1 = h*angle_dot(t12(i),Turn_angle7(i));
     k2 = h*angle_dot(t12(i)+0.5*h,Turn_angle7(i)+0.5*k1);
     k3 = h*angle_dot(t12(i)+0.5*h,Turn_angle7(i)+0.5*k2);
     k4 = h*angle_dot(t12(i)+h,Turn_angle7(i)+k3);
     t12(i+1) = t12(i)+h;
     Turn_angle7(i+1) = Turn_angle7(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
v_turn =v12(i);
t_turn = t12(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity first turn (180 deg) M3 ")
end
%% first cruise
i=1;
v13(i)=v_turn;
t13(i)=t_turn;
distance_cruise(i)=0;
 while distance_cruise <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(t13(i),v13(i));
     k2 = h*v_dot(t13(i)+0.5*h,v13(i)+0.5*k1);
     k3 = h*v_dot(t13(i)+0.5*h,v13(i)+0.5*k2);
     k4 = h*v_dot(t13(i)+h,v13(i)+k3);
     t13(i+1) = t13(i)+h;
     v13(i+1) = v13(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise(i+1) = distance_cruise(i)+(v13(i+1)+v13(i))*0.5*h;
     i=i+1;
 end
v_cruise=v13(i);
t_cruise=t13(i);


%% turn 360
i=1;
high_load_factor = 0;
v14(i) = v_cruise;
t14(i) = t_cruise;
Turn_angle8(i)=0;
 while (Turn_angle8 < 2*pi)
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW_3);
     k1 = h*v_dot(t14(i),v14(i));
     k2 = h*v_dot(t14(i)+0.5*h,v14(i)+0.5*k1);
     k3 = h*v_dot(t14(i)+0.5*h,v14(i)+0.5*k2);
     k4 = h*v_dot(t14(i)+h,v14(i)+k3);
     v14(i+1) = v14(i)+1/6*(k1+2*k2+2*k3+k4);
     
      if v14(i+1) < v_stall
         
         v14(i+1) = v_stall;
         high_load_factor = 1;
     end
     
     
     angle_dot= @(t,Turn_angle8)(9.81*sqrt((n^2)-1)/v14(i+1));
     k1 = h*angle_dot(t14(i),Turn_angle8(i));
     k2 = h*angle_dot(t14(i)+0.5*h,Turn_angle8(i)+0.5*k1);
     k3 = h*angle_dot(t14(i)+0.5*h,Turn_angle8(i)+0.5*k2);
     k4 = h*angle_dot(t14(i)+h,Turn_angle8(i)+k3);
     t14(i+1) = t14(i)+h;
     Turn_angle8(i+1) = Turn_angle8(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
v_turn360 =v14(i);
t_turn360 = t14(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity turn 360 M3 ")
end
%% crusie 2 after turn 360
i=1;
v15(i)=v_turn360;
t15(i)=t_turn360;
distance_cruise2(i)=0;
 while distance_cruise2 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(t15(i),v15(i));
     k2 = h*v_dot(t15(i)+0.5*h,v15(i)+0.5*k1);
     k3 = h*v_dot(t15(i)+0.5*h,v15(i)+0.5*k2);
     k4 = h*v_dot(t15(i)+h,v15(i)+k3);
     t15(i+1) = t15(i)+h;
     v15(i+1) = v15(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise2(i+1) = distance_cruise2(i)+(v15(i+1)+v15(i))*0.5*h;
     i=i+1;
 end
v_cruise2=v15(i);
t_cruise2=t15(i);


%% second turn (180 deg)
i=1;
high_load_factor = 0;
v16(i) = v_cruise2;
t16(i) = t_cruise2;
Turn_angle9(i)=0;
 while Turn_angle9 < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW_3);
     k1 = h*v_dot(t16(i),v16(i));
     k2 = h*v_dot(t16(i)+0.5*h,v16(i)+0.5*k1);
     k3 = h*v_dot(t16(i)+0.5*h,v16(i)+0.5*k2);
     k4 = h*v_dot(t16(i)+h,v16(i)+k3);
     v16(i+1) = v16(i)+1/6*(k1+2*k2+2*k3+k4);
     
      if v16(i+1) < v_stall
         
         v16(i+1) = v_stall;
         high_load_factor = 1;
     end
     
     angle_dot= @(t,Turn_angle9)(9.81*sqrt((n^2)-1)/v16(i+1));
     k1 = h*angle_dot(t16(i),Turn_angle9(i));
     k2 = h*angle_dot(t16(i)+0.5*h,Turn_angle9(i)+0.5*k1);
     k3 = h*angle_dot(t16(i)+0.5*h,Turn_angle9(i)+0.5*k2);
     k4 = h*angle_dot(t16(i)+h,Turn_angle9(i)+k3);
     t16(i+1) = t16(i)+h;
     Turn_angle9(i+1) = Turn_angle9(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end

v_turn2 =v16(i);
t_turn2 = t16(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity second turn (180 deg) M3 ")
end
%% crusie 3 after turn (180 deg)
i=1;
v17(i)=v_turn2;
t17(i)=t_turn2;
distance_cruise3(i)=0;
 while distance_cruise3 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(t17(i),v17(i));
     k2 = h*v_dot(t17(i)+0.5*h,v17(i)+0.5*k1);
     k3 = h*v_dot(t17(i)+0.5*h,v17(i)+0.5*k2);
     k4 = h*v_dot(t17(i)+h,v17(i)+k3);
     t17(i+1) = t17(i)+h;
     v17(i+1) = v17(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise3(i+1) = distance_cruise3(i)+(v17(i+1)+v17(i))*0.5*h;
     i=i+1;
 end
v_cruise3=v17(i);
t_cruise3=t17(i);
time_lap_1=t_cruise3;

%%%%%%%Lap general%%%%%%%
%%%%%%%%%%Lap 2%%%%%%%%%%

%% 1st cruise
i=1;
vG8(i)=v_cruise3;
tG8(i)=t_cruise3;
distanceG_cruise1(i)=0;
 while ( distanceG_cruise1 < 150)
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(tG8(i),vG8(i));
     k2 = h*v_dot(tG8(i)+0.5*h,vG8(i)+0.5*k1);
     k3 = h*v_dot(tG8(i)+0.5*h,vG8(i)+0.5*k2);
     k4 = h*v_dot(tG8(i)+h,vG8(i)+k3);
     tG8(i+1) = tG8(i)+h;
     vG8(i+1) = vG8(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise1(i+1) = distanceG_cruise1(i)+(vG8(i+1)+vG8(i))*0.5*h;
     i=i+1;
 end
 vG_cruise1=vG8(i);
tG_cruise1=tG8(i);
%% first turn (180 deg)
i=1;
high_load_factor = 0;
vG9(i) = vG_cruise1;
tG9(i) = tG_cruise1;
Turn_angle10(i)=0;
 while Turn_angle10 < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW_3);
     k1 = h*v_dot(tG9(i),vG9(i));
     k2 = h*v_dot(tG9(i)+0.5*h,vG9(i)+0.5*k1);
     k3 = h*v_dot(tG9(i)+0.5*h,vG9(i)+0.5*k2);
     k4 = h*v_dot(tG9(i)+h,vG9(i)+k3);
     vG9(i+1) = vG9(i)+1/6*(k1+2*k2+2*k3+k4);
     
      if vG9(i+1) < v_stall
         
         vG9(i+1) = v_stall;
         high_load_factor = 1;
     end
     
     
     angle_dot= @(t,Turn_angle10)(9.81*sqrt((n^2)-1)/vG9(i+1));
     k1 = h*angle_dot(tG9(i),Turn_angle10(i));
     k2 = h*angle_dot(tG9(i)+0.5*h,Turn_angle10(i)+0.5*k1);
     k3 = h*angle_dot(tG9(i)+0.5*h,Turn_angle10(i)+0.5*k2);
     k4 = h*angle_dot(tG9(i)+h,Turn_angle10(i)+k3);
     tG9(i+1) = tG9(i)+h;
     Turn_angle10(i+1) = Turn_angle10(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
vG_turn1 =vG9(i);
tG_turn1 = tG9(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity first turn (180 deg) M3 lap 2 ")
end
%% 2nd cruise
i=1;
vG10(i)=vG_turn1;
tG10(i)=tG_turn1;
distanceG_cruise2(i)=0;
 while distanceG_cruise2 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(tG10(i),vG10(i));
     k2 = h*v_dot(tG10(i)+0.5*h,vG10(i)+0.5*k1);
     k3 = h*v_dot(tG10(i)+0.5*h,vG10(i)+0.5*k2);
     k4 = h*v_dot(tG10(i)+h,vG10(i)+k3);
     tG10(i+1) = tG10(i)+h;
     vG10(i+1) = vG10(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise2(i+1) = distanceG_cruise2(i)+(vG10(i+1)+vG10(i))*0.5*h;
     i=i+1;
 end
 vG_cruise2=vG10(i);
tG_cruise2=tG10(i);

 %% turn 360
i=1;
high_load_factor = 0;
vG11(i) = vG_cruise2;
tG11(i) = tG_cruise2;
Turn_angle11(i)=0;
 while (Turn_angle11 < 2*pi)
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW_3);
     k1 = h*v_dot(tG11(i),vG11(i));
     k2 = h*v_dot(tG11(i)+0.5*h,vG11(i)+0.5*k1);
     k3 = h*v_dot(tG11(i)+0.5*h,vG11(i)+0.5*k2);
     k4 = h*v_dot(tG11(i)+h,vG11(i)+k3);
     vG11(i+1) = vG11(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if vG11(i+1) < v_stall
         
         vG11(i+1) = v_stall;
         high_load_factor = 1;
     end
     
     angle_dot= @(t,Turn_angle11)(9.81*sqrt((n^2)-1)/vG11(i+1));
     k1 = h*angle_dot(tG11(i),Turn_angle11(i));
     k2 = h*angle_dot(tG11(i)+0.5*h,Turn_angle11(i)+0.5*k1);
     k3 = h*angle_dot(tG11(i)+0.5*h,Turn_angle11(i)+0.5*k2);
     k4 = h*angle_dot(tG11(i)+h,Turn_angle11(i)+k3);
     tG11(i+1) = tG11(i)+h;
     Turn_angle11(i+1) = Turn_angle11(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
vG_turn2 =vG11(i);
tG_turn2 = tG11(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity turn 360 M3 lap 2 ")
end
%% crusie 3 after turn 360
i=1;
vG12(i)=vG_turn2;
tG12(i)=tG_turn2;
distanceG_cruise3(i)=0;
 while distanceG_cruise3 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(tG12(i),vG12(i));
     k2 = h*v_dot(tG12(i)+0.5*h,vG12(i)+0.5*k1);
     k3 = h*v_dot(tG12(i)+0.5*h,vG12(i)+0.5*k2);
     k4 = h*v_dot(tG12(i)+h,vG12(i)+k3);
     tG12(i+1) = tG12(i)+h;
     vG12(i+1) = vG12(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise3(i+1) = distanceG_cruise3(i)+(vG12(i+1)+vG12(i))*0.5*h;
     i=i+1;
 end
vG_cruise3=vG12(i);
tG_cruise3=tG12(i);


%% second turn (180 deg)
i=1;
high_load_factor = 0;
vG13(i) = vG_cruise3;
tG13(i) = tG_cruise3;
Turn_angle12(i)=0;
 while Turn_angle12 < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW_3);
     k1 = h*v_dot(tG13(i),vG13(i));
     k2 = h*v_dot(tG13(i)+0.5*h,vG13(i)+0.5*k1);
     k3 = h*v_dot(tG13(i)+0.5*h,vG13(i)+0.5*k2);
     k4 = h*v_dot(tG13(i)+h,vG13(i)+k3);
     vG13(i+1) = vG13(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if vG13(i+1) < v_stall
         
         vG13(i+1) = v_stall;
         high_load_factor = 1;
     end
     
     angle_dot= @(t,Turn_angle12)(9.81*sqrt((n^2)-1)/vG13(i+1));
     k1 = h*angle_dot(tG13(i),Turn_angle12(i));
     k2 = h*angle_dot(tG13(i)+0.5*h,Turn_angle12(i)+0.5*k1);
     k3 = h*angle_dot(tG13(i)+0.5*h,Turn_angle12(i)+0.5*k2);
     k4 = h*angle_dot(tG13(i)+h,Turn_angle12(i)+k3);
     tG13(i+1) = tG13(i)+h;
     Turn_angle12(i+1) = Turn_angle12(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end

vG_turn3 =vG13(i);
tG_turn3 = tG13(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at second turn (180 deg) M3 lap 2 ")
end
%% crusie 4 after turn (180 deg)
i=1;
vG14(i)=vG_turn3;
tG14(i)=tG_turn3;
distanceG_cruise4(i)=0;
 while distanceG_cruise4 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW_3);
     k1 = h*v_dot(tG14(i),vG14(i));
     k2 = h*v_dot(tG14(i)+0.5*h,vG14(i)+0.5*k1);
     k3 = h*v_dot(tG14(i)+0.5*h,vG14(i)+0.5*k2);
     k4 = h*v_dot(tG14(i)+h,vG14(i)+k3);
     tG14(i+1) = tG14(i)+h;
     vG14(i+1) = vG14(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise4(i+1) = distanceG_cruise4(i)+(vG14(i+1)+vG14(i))*0.5*h;
     i=i+1;
 end
vG_cruise4=vG14(i);
tG_cruise4=tG14(i);
time_lap_2=tG_cruise4 - time_lap_1;

%% no_of_laps time

time_remaining = 600-time_lap_1-time_lap_2;

no_of_laps_time= 2 + floor(time_remaining/time_lap_2); % laps from time calculations

%% no_of_laps capacity

bat_cap_M3 = M3.bat_cap;
bat_cap_M3 = 0.8 * bat_cap_M3 ;

time_Lap1_not_include_2Phases_hour = (time_lap_1 - t_climb) / 3600;

cap_consumed_lap1 = cap_consumed_M3_max_throttle + (I_throttle * 1000 * time_Lap1_not_include_2Phases_hour);

time_lap_2_hour = time_lap_2 / 3600;

cap_consumed_lap2 = (I_throttle * 1000 * time_lap_2_hour);

bat_cap_M3 = bat_cap_M3 - cap_consumed_lap2 - cap_consumed_lap1;

no_of_laps_cap = 2 + floor(bat_cap_M3 /cap_consumed_lap2);

% comparing to get possible no_of_laps

no_of_laps = min(no_of_laps_cap ,no_of_laps_time );

if no_of_laps == no_of_laps_cap
    disp(" This no. of laps depending on Capacity")
else
    disp(" This no. of laps depending on Time constraint")
end

%% Total time

Time_M3=(time_lap_1+(no_of_laps-1)*(time_lap_2))/60;  % in minutes

cap_consumed_M3 = cap_consumed_lap1 + ( no_of_laps * cap_consumed_lap2 );
 %% overall plot
figure;

% subplot(1,2,1);
V_overall=[v9 v10 v11 v12 v13 v14 v15 v16 v17 vG8 vG9 vG10 vG11 vG12 vG13 vG14];
T_overall=[t9 t10 t11 t12 t13 t14 t15 t16 t17 tG8 tG9 tG10 tG11 tG12 tG13 tG14];
plot(T_overall,V_overall)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Mission 3')
grid on

x=5;
y=6;

end