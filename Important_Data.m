function Important_Data(wing,flap,tail,CLmax,MTOW,pylod,bat_w,CD0,Vc)

disp('Wing Parameters')
disp(' ')
disp(wing)
disp('Flap Parameters')
disp(' ')
disp(flap)
% disp('Horizontal Tail Parameters')
% disp(' ')
% disp(htail)
disp('Vee Tail Parameters')
disp(' ')
disp(tail)

% Airfoil
Airfoil.Name = wing.Airfoil;
Airfoil.CL_Max = CLmax;
disp('Wing Airfoil Parameters')
disp(' ')
disp(Airfoil)

% Mission 2
Mission.MTOW = MTOW;
Mission.Payload = pylod;
Mission.EmptyWeight = MTOW - pylod - bat_w;
Mission.CDo = CD0;
Mission.Vcruise_avg = Vc;

disp('Mission Main Parameters')
disp(' ')
disp(Mission)

% Mission 3
% 
% Mission3.MTOW = MTOW_3;
% Mission3.Payload = pylod_M3;
% Mission3.EmptyWeight = MTOW_3 - pylod_M3 - bat_w_M3;
% Mission3.CDo = CD0_3;
% Mission3.Vcruise_avg = Vc_3;

% disp('Mission # 3 Main Parameters')
% disp(' ')
% disp(Mission3)
end

