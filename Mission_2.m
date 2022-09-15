function [Time_M2,distance_takeoff,cap_consumed_M2]= Mission_2(wing,cl_max,n,M2,MTOW);

Ts = M2.Ts_100;
I_max_100_M2 = M2.I_max ;
%note that c1,c2 are the coeifficients of Drag
c1 = M2.C1;
c2 = M2.C2;
%note that c3,c4 are the coeifficients of thrust-propulsion
c3 = M2.P1;
c4 = M2.P2;

S_ref = wing.s;
AR = wing.AR;
 % Runge Katta approach
 h=.1;
 i=1;
 t1(i)=0;
 v1(i)=0;
 distance(i) = 0;
 c2_max = 0.5 *1.225*S_ref*cl_max^2/(pi*0.85*AR) ;
 L_takeoff = 0;

%CL turn & c2
CL_turn = 2*n*MTOW*9.81/(1.225*S_ref);
c2_turn = 0.5*1.225*S_ref*CL_turn^2/(pi*0.85*AR);

%%%%%%%%Lap 1%%%%%%%

v_stall = sqrt((2*MTOW*9.806)/(1.225*S_ref*cl_max));
%% takeoff
while (L_takeoff <= (MTOW*9.81))
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_max*v^2-0.05*(0.5*1.225*v^2*S_ref*cl_max-(MTOW*9.81)))/MTOW);
     k1 = h*v_dot(t1(i),v1(i));
     k2 = h*v_dot(t1(i)+0.5*h,v1(i)+0.5*k1);
     k3 = h*v_dot(t1(i)+0.5*h,v1(i)+0.5*k2);
     k4 = h*v_dot(t1(i)+h,v1(i)+k3);
     t1(i+1) = t1(i)+h;
     v1(i+1) = v1(i)+1/6*(k1+2*k2+2*k3+k4);
     L_takeoff = 0.5*1.225*S_ref*(v1(i+1)^2)*cl_max ;
     distance(i+1) = distance(i)+(v1(i+1)+v1(i))*0.5*h;
     i=i+1;

 end
v_takeoff = v1(i);
t_takeoff = t1(i);
distance_takeoff = distance(i);

 %% climb
 i=1;
 altitude(i)=0;
 theta = 25;
 t2(i)=t_takeoff;
 v2(i)=v_takeoff;
 ROC(i)=0;
 while (altitude <= 20 )
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_max*v^2-MTOW*9.81*sind(theta))/MTOW);
     k1 = h*v_dot(t2(i),v2(i));
     k2 = h*v_dot(t2(i)+0.5*h,v2(i)+0.5*k1);
     k3 = h*v_dot(t2(i)+0.5*h,v2(i)+0.5*k2);
     k4 = h*v_dot(t2(i)+h,v2(i)+k3);
     t2(i+1) = t2(i)+h;
     v2(i+1) = v2(i)+1/6*(k1+2*k2+2*k3+k4);
     ROC(i+1) = v2(i+1)*sind(theta);
     altitude(i+1) = altitude(i)+(ROC(i+1)+ROC(i))*0.5*h;
     i=i+1;
 end
v_climb =v2(i);
t_climb = t2(i);
altitude_climb = altitude(i);
distance_climb = altitude_climb/tand(theta);
%% cruise 0 (after climb)
i=1;
v0(i)=v_climb;
t0(i)=t_climb;
distance_cruise0(i)=0;
 while ( distance_cruise0 <( 150-distance_climb-distance_takeoff ))
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(t0(i),v0(i));
     k2 = h*v_dot(t0(i)+0.5*h,v0(i)+0.5*k1);
     k3 = h*v_dot(t0(i)+0.5*h,v0(i)+0.5*k2);
     k4 = h*v_dot(t0(i)+h,v0(i)+k3);
     t0(i+1) = t0(i)+h;
     v0(i+1) = v0(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise0(i+1) = distance_cruise0(i)+(v0(i+1)+v0(i))*0.5*h;
     i=i+1;
 end
v_cruise0=v0(i);
t_cruise0=t0(i);
%% first turn (180 deg)
i=1;
high_load_factor=0;
v3(i) = v_cruise0;
t3(i) = t_cruise0;
Turn_angle(i)=0;
 while Turn_angle < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW);
     k1 = h*v_dot(t3(i),v3(i));
     k2 = h*v_dot(t3(i)+0.5*h,v3(i)+0.5*k1);
     k3 = h*v_dot(t3(i)+0.5*h,v3(i)+0.5*k2);
     k4 = h*v_dot(t3(i)+h,v3(i)+k3);
     v3(i+1) = v3(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if v3(i+1) < v_stall
         
         v3(i+1) = v_stall;
         high_load_factor = 1;
     end
     
     angle_dot= @(t,Turn_angle)(9.81*sqrt((n^2)-1)/v3(i+1));
     k1 = h*angle_dot(t3(i),Turn_angle(i));
     k2 = h*angle_dot(t3(i)+0.5*h,Turn_angle(i)+0.5*k1);
     k3 = h*angle_dot(t3(i)+0.5*h,Turn_angle(i)+0.5*k2);
     k4 = h*angle_dot(t3(i)+h,Turn_angle(i)+k3);
     t3(i+1) = t3(i)+h;
     Turn_angle(i+1) = Turn_angle(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
v_turn = v3(i);
t_turn = t3(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at first turn (180 deg) M2")
end

%% first cruise
i = 1;
v4(i) = v_turn;
t4(i) = t_turn;
distance_cruise(i) = 0;
 while distance_cruise < 150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(t4(i),v4(i));
     k2 = h*v_dot(t4(i)+0.5*h,v4(i)+0.5*k1);
     k3 = h*v_dot(t4(i)+0.5*h,v4(i)+0.5*k2);
     k4 = h*v_dot(t4(i)+h,v4(i)+k3);
     t4(i+1) = t4(i)+h;
     v4(i+1) = v4(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise(i+1) = distance_cruise(i)+(v4(i+1)+v4(i))*0.5*h;
     i = i+1;
 end
v_cruise = v4(i);
t_cruise = t4(i);

%% turn 360
i=1;
high_load_factor=0;
v5(i) = v_cruise;
t5(i) = t_cruise;
Turn_angle2(i)=0;
 while (Turn_angle2 < 2*pi)
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW);
     k1 = h*v_dot(t5(i),v5(i));
     k2 = h*v_dot(t5(i)+0.5*h,v5(i)+0.5*k1);
     k3 = h*v_dot(t5(i)+0.5*h,v5(i)+0.5*k2);
     k4 = h*v_dot(t5(i)+h,v5(i)+k3);
     v5(i+1) = v5(i)+1/6*(k1+2*k2+2*k3+k4);
     
      if v5(i+1) < v_stall
         
         v5(i+1) = v_stall;
         high_load_factor = 1;
        
     end
     
     
     angle_dot= @(t,Turn_angle2)(9.81*sqrt((n^2)-1)/v5(i+1));
     k1 = h*angle_dot(t5(i),Turn_angle2(i));
     k2 = h*angle_dot(t5(i)+0.5*h,Turn_angle2(i)+0.5*k1);
     k3 = h*angle_dot(t5(i)+0.5*h,Turn_angle2(i)+0.5*k2);
     k4 = h*angle_dot(t5(i)+h,Turn_angle2(i)+k3);
     t5(i+1) = t5(i)+h;
     Turn_angle2(i+1) = Turn_angle2(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
v_turn360 =v5(i);
t_turn360 = t5(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at turn 360 M2 ")
end

%% crusie 2 after turn 360
i=1;
v6(i)=v_turn360;
t6(i)=t_turn360;
distance_cruise2(i)=0;
 while distance_cruise2 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(t6(i),v6(i));
     k2 = h*v_dot(t6(i)+0.5*h,v6(i)+0.5*k1);
     k3 = h*v_dot(t6(i)+0.5*h,v6(i)+0.5*k2);
     k4 = h*v_dot(t6(i)+h,v6(i)+k3);
     t6(i+1) = t6(i)+h;
     v6(i+1) = v6(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise2(i+1) = distance_cruise2(i)+(v6(i+1)+v6(i))*0.5*h;
     i=i+1;
 end
v_cruise2=v6(i);
t_cruise2=t6(i);


%% second turn (180 deg)
i=1;
high_load_factor=0;
v7(i) = v_cruise2;
t7(i) = t_cruise2;
Turn_angle3(i)=0;
 while Turn_angle3 < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW);
     k1 = h*v_dot(t7(i),v7(i));
     k2 = h*v_dot(t7(i)+0.5*h,v7(i)+0.5*k1);
     k3 = h*v_dot(t7(i)+0.5*h,v7(i)+0.5*k2);
     k4 = h*v_dot(t7(i)+h,v7(i)+k3);
     v7(i+1) = v7(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if v7(i+1) < v_stall
         
         v7(i+1) = v_stall;
         high_load_factor=1;
         
     end
     
     angle_dot= @(t,Turn_angle3)(9.81*sqrt((n^2)-1)/v7(i+1));
     k1 = h*angle_dot(t7(i),Turn_angle3(i));
     k2 = h*angle_dot(t7(i)+0.5*h,Turn_angle3(i)+0.5*k1);
     k3 = h*angle_dot(t7(i)+0.5*h,Turn_angle3(i)+0.5*k2);
     k4 = h*angle_dot(t7(i)+h,Turn_angle3(i)+k3);
     t7(i+1) = t7(i)+h;
     Turn_angle3(i+1) = Turn_angle3(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end

v_turn2 =v7(i);
t_turn2 = t7(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at second turn (180 deg) M2")
end

%% crusie 3 after turn (180 deg)
i=1;
v8(i)=v_turn2;
t8(i)=t_turn2;
distance_cruise3(i)=0;
 while distance_cruise3 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(t8(i),v8(i));
     k2 = h*v_dot(t8(i)+0.5*h,v8(i)+0.5*k1);
     k3 = h*v_dot(t8(i)+0.5*h,v8(i)+0.5*k2);
     k4 = h*v_dot(t8(i)+h,v8(i)+k3);
     t8(i+1) = t8(i)+h;
     v8(i+1) = v8(i)+1/6*(k1+2*k2+2*k3+k4);
     distance_cruise3(i+1) = distance_cruise3(i)+(v8(i+1)+v8(i))*0.5*h;
     i=i+1;
 end
v_cruise3=v8(i);
t_cruise3=t8(i);
time_lap_1=t_cruise3;

%%%%%%%Lap general%%%%%%%
%%%%%%%%%%Lap 2%%%%%%%%%%

%% 1st cruise
i=1;
vG1(i)=v_cruise3;
tG1(i)=t_cruise3;
distanceG_cruise1(i)=0;
 while ( distanceG_cruise1 < 150)
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(tG1(i),vG1(i));
     k2 = h*v_dot(tG1(i)+0.5*h,vG1(i)+0.5*k1);
     k3 = h*v_dot(tG1(i)+0.5*h,vG1(i)+0.5*k2);
     k4 = h*v_dot(tG1(i)+h,vG1(i)+k3);
     tG1(i+1) = tG1(i)+h;
     vG1(i+1) = vG1(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise1(i+1) = distanceG_cruise1(i)+(vG1(i+1)+vG1(i))*0.5*h;
     i=i+1;
 end
 vG_cruise1=vG1(i);
tG_cruise1=tG1(i);
%% first turn (180 deg)
i=1;
high_load_factor=0;
vG2(i) = vG_cruise1;
tG2(i) = tG_cruise1;
Turn_angle4(i)=0;
 while Turn_angle4 < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW);
     k1 = h*v_dot(tG2(i),vG2(i));
     k2 = h*v_dot(tG2(i)+0.5*h,vG2(i)+0.5*k1);
     k3 = h*v_dot(tG2(i)+0.5*h,vG2(i)+0.5*k2);
     k4 = h*v_dot(tG2(i)+h,vG2(i)+k3);
     vG2(i+1) = vG2(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if vG2(i+1) < v_stall
         
         vG2(i+1) = v_stall;
         high_load_factor=1;
     end
     
     angle_dot= @(t,Turn_angle4)(9.81*sqrt((n^2)-1)/vG2(i+1));
     k1 = h*angle_dot(tG2(i),Turn_angle4(i));
     k2 = h*angle_dot(tG2(i)+0.5*h,Turn_angle4(i)+0.5*k1);
     k3 = h*angle_dot(tG2(i)+0.5*h,Turn_angle4(i)+0.5*k2);
     k4 = h*angle_dot(tG2(i)+h,Turn_angle4(i)+k3);
     tG2(i+1) = tG2(i)+h;
     Turn_angle4(i+1) = Turn_angle4(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
vG_turn1 =vG2(i);
tG_turn1 = tG2(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at first turn (180 deg) M2 lap 2 ")
end
%% 2nd cruise
i=1;
vG3(i)=vG_turn1;
tG3(i)=tG_turn1;
distanceG_cruise2(i)=0;
 while distanceG_cruise2 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(tG3(i),vG3(i));
     k2 = h*v_dot(tG3(i)+0.5*h,vG3(i)+0.5*k1);
     k3 = h*v_dot(tG3(i)+0.5*h,vG3(i)+0.5*k2);
     k4 = h*v_dot(tG3(i)+h,vG3(i)+k3);
     tG3(i+1) = tG3(i)+h;
     vG3(i+1) = vG3(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise2(i+1) = distanceG_cruise2(i)+(vG3(i+1)+vG3(i))*0.5*h;
     i=i+1;
 end
 vG_cruise2=vG3(i);
tG_cruise2=tG3(i);

 %% turn 360
i=1;
high_load_factor=0;
vG4(i) = vG_cruise2;
tG4(i) = tG_cruise2;
Turn_angle5(i)=0;
 while (Turn_angle5 < 2*pi)
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW);
     k1 = h*v_dot(tG4(i),vG4(i));
     k2 = h*v_dot(tG4(i)+0.5*h,vG4(i)+0.5*k1);
     k3 = h*v_dot(tG4(i)+0.5*h,vG4(i)+0.5*k2);
     k4 = h*v_dot(tG4(i)+h,vG4(i)+k3);
     vG4(i+1) = vG4(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if vG4(i+1) < v_stall
         
         vG4(i+1) = v_stall;
         high_load_factor=1;
         
     end
     
     
     angle_dot= @(t,Turn_angle5)(9.81*sqrt((n^2)-1)/vG4(i+1));
     k1 = h*angle_dot(tG4(i),Turn_angle5(i));
     k2 = h*angle_dot(tG4(i)+0.5*h,Turn_angle5(i)+0.5*k1);
     k3 = h*angle_dot(tG4(i)+0.5*h,Turn_angle5(i)+0.5*k2);
     k4 = h*angle_dot(tG4(i)+h,Turn_angle5(i)+k3);
     tG4(i+1) = tG4(i)+h;
     Turn_angle5(i+1) = Turn_angle5(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end
vG_turn2 =vG4(i);
tG_turn2 = tG4(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at turn 360 M2 lap 2 ")
end

%% crusie 3 after turn 360
i=1;
vG5(i)=vG_turn2;
tG5(i)=tG_turn2;
distanceG_cruise3(i)=0;
 while distanceG_cruise3 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(tG5(i),vG5(i));
     k2 = h*v_dot(tG5(i)+0.5*h,vG5(i)+0.5*k1);
     k3 = h*v_dot(tG5(i)+0.5*h,vG5(i)+0.5*k2);
     k4 = h*v_dot(tG5(i)+h,vG5(i)+k3);
     tG5(i+1) = tG5(i)+h;
     vG5(i+1) = vG5(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise3(i+1) = distanceG_cruise3(i)+(vG5(i+1)+vG5(i))*0.5*h;
     i=i+1;
 end
vG_cruise3=vG5(i);
tG_cruise3=tG5(i);


%% second turn (180 deg)
i=1;
high_load_factor=0;
vG6(i) = vG_cruise3;
tG6(i) = tG_cruise3;
Turn_angle6(i)=0;
 while Turn_angle6 < pi
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2_turn/v^2)/MTOW);
     k1 = h*v_dot(tG6(i),vG6(i));
     k2 = h*v_dot(tG6(i)+0.5*h,vG6(i)+0.5*k1);
     k3 = h*v_dot(tG6(i)+0.5*h,vG6(i)+0.5*k2);
     k4 = h*v_dot(tG6(i)+h,vG6(i)+k3);
     vG6(i+1) = vG6(i)+1/6*(k1+2*k2+2*k3+k4);
     
     if vG6(i+1) < v_stall
         
         vG6(i+1) = v_stall;
         high_load_factor=1;
     end
     
     
     
     angle_dot= @(t,Turn_angle6)(9.81*sqrt((n^2)-1)/vG6(i+1));
     k1 = h*angle_dot(tG6(i),Turn_angle6(i));
     k2 = h*angle_dot(tG6(i)+0.5*h,Turn_angle6(i)+0.5*k1);
     k3 = h*angle_dot(tG6(i)+0.5*h,Turn_angle6(i)+0.5*k2);
     k4 = h*angle_dot(tG6(i)+h,Turn_angle6(i)+k3);
     tG6(i+1) = tG6(i)+h;
     Turn_angle6(i+1) = Turn_angle6(i)+1/6*(k1+2*k2+2*k3+k4);
     i = i+1;
 end

vG_turn3 =vG6(i);
tG_turn3 = tG6(i);

if high_load_factor == 1
    disp(" Adjust the load factor, the velocity at turn is smaller than stall velocity at second turn (180 deg) M2 lap 2 ")
end
%% crusie 4 after turn (180 deg)
i=1;
vG7(i)=vG_turn3;
tG7(i)=tG_turn3;
distanceG_cruise4(i)=0;
 while distanceG_cruise4 <150
     v_dot = @(t,v)((((Ts-c3*v^2-c4*v)*0.001*9.81)-c1*v^2-c2/v^2)/MTOW);
     k1 = h*v_dot(tG7(i),vG7(i));
     k2 = h*v_dot(tG7(i)+0.5*h,vG7(i)+0.5*k1);
     k3 = h*v_dot(tG7(i)+0.5*h,vG7(i)+0.5*k2);
     k4 = h*v_dot(tG7(i)+h,vG7(i)+k3);
     tG7(i+1) = tG7(i)+h;
     vG7(i+1) = vG7(i)+1/6*(k1+2*k2+2*k3+k4);
     distanceG_cruise4(i+1) = distanceG_cruise4(i)+(vG7(i+1)+vG7(i))*0.5*h;
     i=i+1;
 end
vG_cruise4=vG7(i);
tG_cruise4=tG7(i);
time_lap_2=tG_cruise4 - t_cruise3;

%% Total time

Time_M2=(time_lap_1+(2*(time_lap_2)))/60;
Time_M2_hour = Time_M2 / 60;
cap_consumed_M2= (I_max_100_M2 * 1000 * Time_M2_hour)/0.85;

%% overall plot
figure;

%subplot(1,2,1);
V_overall=[v1 v2 v0 v3 v4 v5 v6 v7 v8 vG1 vG2 vG3 vG4 vG5 vG6 vG7];
T_overall=[t1 t2 t0 t3 t4 t5 t6 t7 t8 tG1 tG2 tG3 tG4 tG5 tG6 tG7];
plot(T_overall,V_overall)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Mission 2')
grid on

end