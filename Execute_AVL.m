  function Execute_AVL(v_cr)

filename = "Aircraft";

% Deleting Old Files

[status,cmdout] = system("del" + " " + filename + ".st");
[status,cmdout] = system("del" + " " + filename + ".fs");
[status,cmdout] = system("del" + " " + "command_file.txt");

%% Create Command File

%Open the file with write permission
fid = fopen("command_file.txt", 'w');

%Load the Geometry File
fprintf(fid, "load" + " " + filename + ".avl\n");

%Load Mass
fprintf(fid, "mass" + " " + filename + ".mass\n");

%Apply Mass Parameters
fprintf(fid, "MSET\n\n"); 

%Disable Graphics
fprintf(fid, "PLOP\ng\n\n"); 

%Open the OPER menu
fprintf(fid, "OPER\n");   

%Define the constraints
fprintf(fid, "c1\n");
velocity = num2str(v_cr);
fprintf(fid,"v" + " " + velocity + "\n\n");

%Set Pitching Moment to 0 for Elevator
fprintf(fid, "d1" + " " + "pm" + " " + "0\n"); 

%Run
fprintf(fid, "x\n"); 

%Extract .ST and .FS files
fprintf(fid, "st\n"); 
fprintf(fid, filename + ".st\n");   
fprintf(fid, "fs\n"); 
fprintf(fid, filename + ".fs\n");  

%Go back to Mainmeun
fprintf(fid, "\n");

%Quit
fprintf(fid, "Quit\n"); 

%Close File
fclose(fid);

%% Execute AVL

[status,cmdout] = system('"avl" < command_file.txt');

end