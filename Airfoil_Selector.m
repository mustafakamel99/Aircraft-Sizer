function [Airfoil,CLmax,Tc_w] = Airfoil_Selector(MTOW,s,MAC,AR,criteria,clmax_min_3D,AOA_c,d_spar,v_cr)

%% Some Parameters

rho = 1.225;
alfa_L_D = 2;            % AOA at which to compare L/D
w = MTOW;

%% Calculating Requirements

clc_3d = w*9.81/(0.5*1.225*v_cr^2*s)
cl_c = clc_3d/(1-clc_3d/pi/AR);      %Cruise Lift Coefficient (2D)

clmax_min = clmax_min_3D/(1-clmax_min_3D/pi/AR)      %Minimum CLmax 2D

c_spar = 5/1000;             % Spar Clearnace (mm) = /1000
d = d_spar/1000 + 2*c_spar;
t_min = d/MAC * 100;                   %Minimum required thickness (%)

%% Reading Database 

filename = "Airfoil_Polar_Data_Re_300000.xlsx";

[airfoil.name,airfoil.thickness] = readvars(filename);
CL = readmatrix(filename,'sheet','2 - CL');
CD = readmatrix(filename,'sheet','3 - CD');
CM = readmatrix(filename,'sheet','4 - CM');
L_D = readmatrix(filename,'sheet','5 - L_D');

%% Filteration (1) - based on Thickness for Spar Requirement

i_1 = [];
m = 1;
for n = 1:length(airfoil.name)
        
    if airfoil.thickness(n) >= t_min
        
        i_1(m) = n;    %Index of accepted airfoils after 1st filter
        m = m+1;
        
    else
    end
end

% Error msg if no airfoils were found

if length(i_1) == 0
    error('Filter (1) - No Airfoils satisfy your Spar Requirement')
    return
else
end

    for index = 1:length(i_1)
        airfoils_after_filter_1(index) = airfoil.name(i_1(index));
    end
    
    file = fopen('filter_1.txt');
    fprintf(file,'%s\n',airfoils_after_filter_1{:});
    fclose(file);
    
%% Filteration (2) - based on Cruise CL @ Desired Trim AOA limits

i_2 = [];
m = 1;
for n = 1:length(i_1)
    i = i_1(n);
    if cl_c >= CL(AOA_c(1)+6,i) && cl_c <= CL(AOA_c(2)+6,i)
        i_2(m) = i;      %Index of accepted airfoils after 3rd filter
        m = m+1;  
    else
    end
end

% Error msg if no airfoils were found

if length(i_2) == 0
    error('Filter (2) - No Airfoils satisfy your Cruise Requirement')
    return
else
end

    for index = 1:length(i_2)
        airfoils_after_filter_2(index) = airfoil.name(i_2(index));
    end
    
    file = fopen('filter_2.txt');
    fprintf(file,'%s\n',airfoils_after_filter_2{:});
    fclose(file);
    
%% Filteration (3) - based on minimum CLmax

i_3 = [];
m = 1;
for n = 1:length(i_2)
    i = i_2(n);
    A = CL(1:end,i);
    if max(A) >= clmax_min 
        i_3(m) = i;      %Index of accepted airfoils after 3rd filter
        m = m+1;  
    else
    end
end

% Error msg if no airfoils were found

if length(i_3) == 0
    error('Filter (3) - No Airfoils satisfy your min CLmax condition')
    return
else
end

    for index = 1:length(i_3)
        airfoils_after_filter_3(index) = airfoil.name(i_3(index));
    end
    
    file = fopen('filter_3.txt');
    fprintf(file,'%s\n',airfoils_after_filter_3{:});
    fclose(file);
    
%% Selecting Airfoil based on Criteria | Final Filter

if criteria == 1    %Based on CLmax
    
    for n = 1:length(i_3)
        i = i_3(n);
        if n==1     
            i_f = i;        %Index of final selected airfoil          
        elseif n > 1            
            A = CL(1:end,i);
            B = CL(1:end,i_f);
            if max(A) > max(B)            
                i_f = i;                
            else                                              
            end
        end
    end
    
elseif criteria == 2    %Based on CDmin
    
    for n = 1:length(i_3)
        i = i_3(n);
        if n==1        
            i_f = i;            
        elseif n > 1            
            A = CD(1:end,i);
            B = CD(1:end,i_f);
            if min(A(A>0)) < min(B(B>0))            
                i_f = i;                
            else                                              
            end
        end
    end
    
elseif criteria == 3    %Based on L/D max @ AOA = 2
    
    for n = 1:length(i_3)
        i = i_3(n);
        if n==1        
            i_f = i;            
        elseif n > 1            
            A = L_D(alfa_L_D + 1 , i);
            B = L_D(alfa_L_D + 1 , i_f);            
            if A > B            
                i_f = i;               
            else                                               
            end
        end
    end
end

%% Displaying Final Selected Airfoil Data
 
Airfoil = airfoil.name(i_f);
A = CL(1:end,i_f);
CLmax = max(A)/(1+max(A)/pi/AR);
Tc_w =airfoil.thickness(i_f);

end