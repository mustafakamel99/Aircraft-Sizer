function [X_cg,i_t,Alpha_trim_M,Cm_a_M,X_np ] = Stability_Trim(MTOW,wing,htail,vtail,tail,v_cr,SM)

%% Main Design Mission

X_cg=0.25*wing.MAC;
i_t=0;
Elevator=1;
itr = 1;
xhinge = 0;
while abs(Elevator)>0.1
    iteration = num2str(itr);
    elevator_string = num2str(Elevator);
    disp("AVL iteration No. =  " + iteration + " ... " + "   " + "Current Elevator Value =  " + elevator_string )
    AVL_Input(MTOW,X_cg,wing,htail,vtail,tail,i_t,xhinge)
    Execute_AVL(v_cr)
    [X_np,Elevator,Alpha_trim,Cm_a] = Read_ST;
    X_cg=X_np-(SM*wing.MAC)/100;
    i_t=i_t+Elevator;
    itr = itr+1;
end

%% Other Mission

% if MTOW == MTOW_2    
%     MTOW_other = MTOW_3;
% else
%     MTOW_other = MTOW_2;    
% end
%     xhinge = 0.8;
%     AVL_Input(MTOW_other,X_cg,wing,htail,vtail,tail,i_t,xhinge)
%     Execute_AVL(v_cr_other)
%     [X_np_other,Elevator_other,Alpha_trim_other,Cm_a_other] = Read_ST;
 
 %% Assigning to M2 and M3
 
% if MTOW == MTOW
%         
     Alpha_trim_M = Alpha_trim;
%     Alpha_trim_M3 = Alpha_trim_other;
     Cm_a_M = Cm_a;
%     Cm_a_M3 = Cm_a_other;
%     
%  else
%     
% %     Alpha_trim_M2 = Alpha_trim_other;
% %     Alpha_trim_M3 = Alpha_trim;
% %     Cm_a_M2 = Cm_a_other;
% %     Cm_a_M3 = Cm_a;
%     
%  end

end