function [Yle,ccl,cl] = Wing_Distribution(wing)

% Open the .fs file as Read

fid = fopen('Aircraft.fs', 'r') ;              % Open source file.

% Skip 20 lines

fgetl(fid) ;                                
fgetl(fid) ;                                  
fgetl(fid) ;                                  
fgetl(fid) ;                                  
fgetl(fid) ;                                  
fgetl(fid) ;                                  
fgetl(fid) ;                                 
fgetl(fid) ;                                 
fgetl(fid) ;                                  
fgetl(fid) ;                                 
fgetl(fid) ;                                
fgetl(fid) ;                                  
fgetl(fid) ;                                  
fgetl(fid) ;                                 
fgetl(fid) ;                                 
fgetl(fid) ;                                  
fgetl(fid) ;                                 
fgetl(fid) ;                                  
fgetl(fid) ;                                  
fgetl(fid) ;                                  

% Read 2320 Characters and store them in ( text )
text = fread(fid, 2320) ;                    

% Close File
fclose(fid);


% Create new file
fid = fopen('Aircraft.txt', 'w') ; 

% Copy (text) to that file
fwrite(fid, text);

% Close File
fclose(fid);

% Wing_Distribution


X= load('Aircraft.txt');

Yle = X(:,2);
ccl = X(:,5)/wing.MAC;
cl = X(:,8);

Yle = [flip(-Yle);Yle];
ccl = [flip(ccl);ccl];
cl = [flip(cl);cl];
end

