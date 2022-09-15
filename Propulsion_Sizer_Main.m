function [C1_M2,C2_M2,C1_M3,C2_M3,Ts_100_M2,Ts_100_M3,Ts_M3,I_max_100_M2,I_max_100_M3,I_throttle,C1_M3_100,C2_M3_100] = ...
    Propulsion_Sizer_Main(v_c_min_M2,v_c_min_M3,Ts_min_M2,Ts_min_M3,T_dyn_min_M2,T_dyn_min_M3,P1_M2,P2_M2,P1_M3,P2_M3,...
     p_min,p_max,bat_v_M2,bat_v_M3,bat_cap_M2,bat_cap_M3,thr_set,fueslage_w,fueslage_h) 
%% Unit Conversion (Newton to gram)

Ts_min_M2 = Ts_min_M2/9.81*1000;
Ts_min_M3 = Ts_min_M3/9.81*1000;
T_dyn_min_M2 = T_dyn_min_M2/9.81*1000;
T_dyn_min_M3 = T_dyn_min_M3/9.81*1000;
P1_M2 = P1_M2/9.81*1000;
P2_M2 = P2_M2/9.81*1000;
P1_M3 = P1_M3/9.81*1000;
P2_M3 = P2_M3/9.81*1000;

%% Desired Values for Scoring Factor Forumla

Delta_Vmax_Acceptable = 5;
Vmax_desired_M2 = 35;
Vmax_desired_M3 = 25;

%% Importing Databases

[ID_Motor,Manu,Model,GR,Kv,Kt,Rm,i0,Max_p_in,Max_v,Weight]= readvars('Motor_Database(mini).xlsx');
[ID_Prop,D,P,B,Max_RPM_prop]= readvars('pitch_ratio_Propeller_Database.xlsx');

%% Thrust Efficiency Calculation

fuslage_d = (fueslage_w + fueslage_h)/2; % we consider the average value between fuselage width and height as the fuselage diameter
fuslage_D = fuslage_d*5000/127;        % converting meter to inch
B_R = fuslage_D./D;                    % the ratio between fuselage diameter & propeller diameter
 
 for n = 1:length(D)
     if (B_R(n) < 0.47)      % if the value of Etta_Th is bigger than 1, the value will be equal 1
         Etta_Th(n,1) = 1;   % Real Static thrust. 
     else
         Etta_Th(n,1) = -log(B_R(n)-0.1);  % the eqn of thrust efficiency
     end
 end

%% Combo Selection Process
i=1;

Final_Combo_Matrix = []; 
  
 for n = 1 : length(ID_Motor)

    if ( Max_v(n) >= bat_v_M2 && Max_v(n) >= bat_v_M3 )           
        
        % Mission # 2
        
       [Max_RPM_M2,I_max_100_M2,Ts_100,T_dyn_100,time_100,p_max_M2,Vmax,eta,Pout,C1_M2,C2_M2]= combo_evaluator_M2...
        (GR(n),Kv(n),Kt(n),Rm(n),i0(n),D,P,B,Etta_Th,bat_v_M2,bat_cap_M2,v_c_min_M2,P1_M2,P2_M2); 
        
        Performance_Matrix_M2 = [ID_Prop, time_100, T_dyn_100, Ts_100, Max_RPM_M2, p_max_M2,I_max_100_M2,Vmax,eta,Pout/Weight(n)];
        
        TF1 = Performance_Matrix_M2(:,2) > 1.5 == 0;
        TF2 = Performance_Matrix_M2(:,2) < 5 == 0;
        TF3 = Performance_Matrix_M2(:,3) > T_dyn_min_M2 == 0;
        TF4 = Performance_Matrix_M2(:,4) > Ts_min_M2 == 0;
        TF5 = Performance_Matrix_M2(:,5) < 0.8*Max_RPM_prop == 0;
        TF6 = Performance_Matrix_M2(:,6) < 0.8*Max_p_in(n) == 0;
        TF7 = imag(Performance_Matrix_M2(:,8)) == 0 == 0;
        
        TF = or(TF1,TF2);
        TF = or(TF,TF3);
        TF = or(TF,TF4);
        TF = or(TF,TF5);
        TF = or(TF,TF6);
        TF = or(TF,TF7);
        
        Performance_Matrix_M2(TF,:) = [];
        
        % Filter No. 1 - According to Vmax
        
        TF = abs(Vmax_desired_M2 - Performance_Matrix_M2(:,8)) <= Delta_Vmax_Acceptable == 0;
        Performance_Matrix_M2(TF,:) = []; 

       if isempty(Performance_Matrix_M2) == 1           
          
       else

       M2_Combo_Matrix = [];            
       M2_Combo_Matrix(:,1) = ID_Motor(n)*ones(length(Performance_Matrix_M2(:,1)),1);
       M2_Combo_Matrix(:,2) = Performance_Matrix_M2(:,1);
       M2_Combo_Matrix(:,3) = Performance_Matrix_M2(:,5);
       M2_Combo_Matrix(:,4) = Performance_Matrix_M2(:,4);
       M2_Combo_Matrix(:,5) = Performance_Matrix_M2(:,7);
       M2_Combo_Matrix(:,6) = Performance_Matrix_M2(:,8);
       M2_Combo_Matrix(:,7) = Performance_Matrix_M2(:,2);
       M2_Combo_Matrix(:,8) = Performance_Matrix_M2(:,3);
       M2_Combo_Matrix(:,9) = Performance_Matrix_M2(:,9);
       M2_Combo_Matrix(:,10) = Performance_Matrix_M2(:,10);
           
       end

       % Mission # 3

            [Max_RPM_M3,I_max_100_M3,Ts_100,T_dyn_M3,time_M3,p_max_M3,I_throttle,Vmax,eta,Pout,C1_M3,C2_M3,Ts_M3,C1_100,C2_100]= combo_evaluator_M3...
             (GR(n),Kv(n),Kt(n),Rm(n),i0(n),D,P,B,Etta_Th,bat_v_M3,bat_cap_M3,v_c_min_M3,thr_set,P1_M3,P2_M3);
                  
        Performance_Matrix_M3 = [ID_Prop, time_M3, T_dyn_M3, Ts_100, Max_RPM_M3, p_max_M3,I_throttle,Vmax,eta,Pout/Weight(n)];
        
        TF1 = Performance_Matrix_M3(:,2) > 8 == 0;
        TF2 = Performance_Matrix_M3(:,2) < 15 == 0;
        TF3 = Performance_Matrix_M3(:,3) > T_dyn_min_M3 == 0;
        TF4 = Performance_Matrix_M3(:,4) > Ts_min_M3 == 0;
        TF5 = Performance_Matrix_M3(:,5) < 0.8*Max_RPM_prop == 0;
        TF6 = Performance_Matrix_M3(:,6) < 0.8*Max_p_in(n) == 0;
        TF7 = imag(Performance_Matrix_M3(:,8)) == 0 == 0;
        
        TF = or(TF1,TF2);
        TF = or(TF,TF3);
        TF = or(TF,TF4);
        TF = or(TF,TF5);
        TF = or(TF,TF6);
        
        Performance_Matrix_M3(TF,:) = [];

        % Filter No. 1 - According to Vmax
        
        TF = abs(Vmax_desired_M3 - Performance_Matrix_M3(:,8)) <= Delta_Vmax_Acceptable == 0;
        Performance_Matrix_M3(TF,:) = [];
         
       if isempty(Performance_Matrix_M3) == 1
           
       else
           
       M3_Combo_Matrix = [];    
       M3_Combo_Matrix(:,1) = ID_Motor(n)*ones(length(Performance_Matrix_M3(:,1)),1);
       M3_Combo_Matrix(:,2) = Performance_Matrix_M3(:,1);
       M3_Combo_Matrix(:,3) = Performance_Matrix_M3(:,5);
       M3_Combo_Matrix(:,4) = Performance_Matrix_M3(:,4);
       M3_Combo_Matrix(:,5) = Performance_Matrix_M3(:,7);
       M3_Combo_Matrix(:,6) = Performance_Matrix_M3(:,8);
       M3_Combo_Matrix(:,7) = Performance_Matrix_M3(:,2);
       M3_Combo_Matrix(:,8) = Performance_Matrix_M3(:,3);
       M3_Combo_Matrix(:,9) = Performance_Matrix_M3(:,9);
       M3_Combo_Matrix(:,10) = Performance_Matrix_M3(:,10);

       end
       
       % Combining M2 and M3 Combos
       
       if isempty(Performance_Matrix_M2) == 1 || isempty(Performance_Matrix_M3) == 1
          
           
       else

        [Best_Power_to_Weight_M2 ,Index_Best_Power_to_Weight_M2] = max(M2_Combo_Matrix(:,10));
        [Best_Power_to_Weight_M3 ,Index_Best_Power_to_Weight_M3] = max(M3_Combo_Matrix(:,10));

        Final_Combo_Matrix(i,1)= ID_Motor(n);
        Final_Combo_Matrix(i,2)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,2);
        Final_Combo_Matrix(i,3)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,2);
        Final_Combo_Matrix(i,4)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,3);
        Final_Combo_Matrix(i,5)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,4);
        Final_Combo_Matrix(i,6)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,5);
        Final_Combo_Matrix(i,7)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,6);
        Final_Combo_Matrix(i,8)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,7);
        Final_Combo_Matrix(i,9)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,8);
        Final_Combo_Matrix(i,10)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,9);
        Final_Combo_Matrix(i,11)= M2_Combo_Matrix(Index_Best_Power_to_Weight_M2,10);
        Final_Combo_Matrix(i,12)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,3);
        Final_Combo_Matrix(i,13)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,4);
        Final_Combo_Matrix(i,14)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,5);
        Final_Combo_Matrix(i,15)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,6);
        Final_Combo_Matrix(i,16)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,7);
        Final_Combo_Matrix(i,17)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,8);
        Final_Combo_Matrix(i,18)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,9);
        Final_Combo_Matrix(i,19)= M3_Combo_Matrix(Index_Best_Power_to_Weight_M3,10);
        Final_Combo_Matrix(i,20)= Final_Combo_Matrix(i,19) + Final_Combo_Matrix(i,11);
        Final_Combo_Matrix(i,21)= Weight(n);
        i = i + 1;         
       end
    end 
  end

%% Finding Best Overall Combination

    if isempty(Final_Combo_Matrix) == 1
        error('NO Motor / Propeller Combinations Found :(')   
    else
        
    % Sorting function 
      
    Enass_Matrix = sortrows(Final_Combo_Matrix,20, 'descend');   % sorted matrix
      
    if (length(Enass_Matrix(:,1))> 10)   
          
         Mustafa_Matrix = Enass_Matrix(1:10,:); % First 10 Rows in Sorted matrix
         [status,cmdout] = system("del" + " " + "Tarek.xlsx");
         xlswrite('Tarek.xlsx',Mustafa_Matrix)
         [Best_Combo_SF ,Index_Best_Combo] = min(Mustafa_Matrix(:,21));
         ID_Motor_Top = Mustafa_Matrix(Index_Best_Combo,1);
         ID_Prop_M2 = Mustafa_Matrix(Index_Best_Combo,2);
         ID_Prop_M3 = Mustafa_Matrix(Index_Best_Combo,3);
         disp("Selected Motor for M2 & M3 :   " + Manu(ID_Motor_Top)+ "   " + Model(ID_Motor_Top))
         
      else
          
        [status,cmdout] = system("del" + " " + "Tarek.xlsx");
        xlswrite('Tarek.xlsx',Enass_Matrix)
        [Best_Combo_SF ,Index_Best_Combo] = min(Enass_Matrix(:,21));
        ID_Motor_Top = Enass_Matrix(Index_Best_Combo,1);
        ID_Prop_M2 = Enass_Matrix(Index_Best_Combo,2);
        ID_Prop_M3 = Enass_Matrix(Index_Best_Combo,3);
        disp("Selected Motor for M2 & M3 :   " + Manu(ID_Motor_Top)+ "   " + Model(ID_Motor_Top))
        
      end

    D_M2 = num2str(D(ID_Prop_M2));
    P_M2 = num2str(P(ID_Prop_M2));
    D_M3 = num2str(D(ID_Prop_M3));
    P_M3 = num2str(P(ID_Prop_M3));

    disp("Selected Propeller for M2 :   " + D_M2 + "x" + P_M2 +"E")
    disp("Selected Propeller for M3 :   " + D_M3 + "x" + P_M3 +"E")

    % Mission 2 Combo Results
    
    [Max_RPM_M2,I_max_100_M2,Ts_100_M2,T_dyn_100,time_100,p_max_M2,Vmax,eta_M2,Pout,C1_M2,C2_M2]= ...
        combo_evaluator_M2(GR(ID_Motor_Top),Kv(ID_Motor_Top),Kt(ID_Motor_Top),Rm(ID_Motor_Top),i0(ID_Motor_Top),...
         D(ID_Prop_M2),P(ID_Prop_M2),B(ID_Prop_M2),Etta_Th(ID_Prop_M2),bat_v_M2,bat_cap_M2,v_c_min_M2,P1_M2,P2_M2);
      
     Result_M2 = table(Max_RPM_M2,Ts_100_M2,I_max_100_M2,Vmax,time_100,T_dyn_100,eta_M2);
      disp("  ")
      disp("Mission 2 combination results:   ")
      disp("  ")
      disp(Result_M2)
      
    % Mission 3 Combo Results

    [Max_RPM_M3,I_max_100_M3,Ts_100_M3,T_dyn_M3,time_M3,p_max_M3,I_throttle,Vmax,eta_M3,Pout,C1_M3,C2_M3,Ts_M3,C1_M3_100,C2_M3_100]= ...
        combo_evaluator_M3(GR(ID_Motor_Top),Kv(ID_Motor_Top),Kt(ID_Motor_Top),Rm(ID_Motor_Top),i0(ID_Motor_Top),D(ID_Prop_M3),...
         P(ID_Prop_M3),B(ID_Prop_M3),Etta_Th(ID_Prop_M3),bat_v_M3,bat_cap_M3,v_c_min_M3,thr_set,P1_M3,P2_M3);
   
      Result_M3 = table(Max_RPM_M3,Ts_100_M3,I_max_100_M3,I_throttle,Vmax,time_M3,T_dyn_M3,eta_M3);
      disp("  ")
      disp("Mission 3 combination results:   ")
      disp("  ")
      disp(Result_M3)
      
    end
end