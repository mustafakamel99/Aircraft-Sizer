function [X_np,Elevator,Alpha_trim,Cm_a] = Read_ST

file = fileread('Aircraft.st');

X_np_str = regexp(file,'(?<=Neutral\spoint\s\sXnp\s=\s\s\s)(.\d*.?\d*)','match');
X_np = str2double(X_np_str);

Elevator_str = regexp(file,'(?<=elevator\s\s\s\s\s\s\s\s=\s\s)(.\d*.?\d*)','match');
Elevator = str2double(Elevator_str);

Alpha_str = regexp(file,'(?<=Alpha\s=\s\s)(.\d*.?\d*)','match');
Alpha_trim = str2double(Alpha_str);
    
Cm_a_str = regexp(file,'(?<=Cma\s=\s\s)(.\d*.?\d*)','match');
Cm_a = str2double(Cm_a_str);

end