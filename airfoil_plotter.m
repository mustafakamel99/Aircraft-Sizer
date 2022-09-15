function [dataX,dataY] = airfoil_plotter(wing)
% Open the .dat file as Read

a_f = fopen([char(wing.Airfoil) '.dat'], 'r') ;              % Open data file.

% Skip 1st line

fgetl(a_f) ;                                                               

% Read all Characters and store them in ( data )
data = fread(a_f) ;                    

% Close File
fclose(a_f);

% Create new file
a_f = fopen('airfoil_used.dat', 'w') ; 

% Copy (text) to that file
fwrite(a_f, data);

% Close File
fclose(a_f);
%%
A = load ('airfoil_used.dat');
[i,j] = size(A); %[i,j]=size(A)=>assigns values relative to 'i' and 'j' 
%                with the array dimension 
m = 1:i; %creates an array using the variation '1:i'
%% Creates the vectors relative to the columns (j=1) and (j=2)
dataX=A(m,1);
dataY=A(m,2);
end