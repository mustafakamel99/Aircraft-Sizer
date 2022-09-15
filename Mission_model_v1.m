 function [Time_for_M2,distance_takeoff_M2,Time_for_M3,distance_takeoff_M3,no_of_laps_M3,Score_M2,Score_M3,...
     Overall_score]= Mission_model_v1(wing,cl_max,n,M2,MTOW_2,M3,score,syringes);

M2.P1 = -1 * M2.C1_M2_Th;
M2.P2 = -1 * M2.C2_M2_Th;
M3.P1_throttle = -1 * M3.C1_M3_Th;
M3.P2_throttle = -1 * M3.C2_M3_Th;
M3.P1_max = -1 * M3.C1_M3_100;
M3.P2_max = -1 * M3.C2_M3_100;
MTOW_3 = M3.MTOW_3;

%% calling functions

%% functions for missions :

[Time_for_M2,distance_takeoff_M2,cap_consumed_M2]= Mission_2(wing,cl_max,n,M2,MTOW_2); %time of M2 in minute


[no_of_laps_M3,Time_for_M3,distance_takeoff_M3,cap_consumed_M3]= Mission_3(wing,cl_max,n,M3);



%% SCORE
max_M2 = score.max_score_M2 ;
max_M3 = score.max_score_M3 ;
report_score = score.report_score ;

Score_M1 = 1;
Score_M2 = 1 + ((syringes.n/Time_for_M2)/(max_M2));
Score_M3 = 2 + ((no_of_laps_M3)/max_M3);

Overall_score = report_score * (Score_M1 + Score_M2 + Score_M3);

%% Remaining capacity

remaining_cap_M2_mAh = M2.bat_cap-cap_consumed_M2;
remaining_cap_M3_mAh = M3.bat_cap-cap_consumed_M3;
remaining_cap_M2_percent = ((remaining_cap_M2_mAh)/(M2.bat_cap)) * 100;      % percentage
remaining_cap_M3_percent = ((remaining_cap_M3_mAh)/(M3.bat_cap)) * 100;

%% Display result

Result_mission_model = table(Time_for_M2,distance_takeoff_M2,Score_M2,syringes.n,no_of_laps_M3,...
    Time_for_M3,distance_takeoff_M3,Score_M3,Overall_score);

      disp("  ")
      disp("Mission model results:   ")
      disp("  ")
      disp(Result_mission_model)
      disp("  ")
      
      Result_capacity = table(remaining_cap_M2_percent,remaining_cap_M2_mAh,...
          remaining_cap_M3_percent,remaining_cap_M3_mAh);
      
      disp("capacity results:   ")
      disp("  ")
      disp(Result_capacity)

 end