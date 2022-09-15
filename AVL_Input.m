function AVL_Input(MTOW,X_cg,wing,htail,vtail,tail,i_t,xhinge)

%% Mass File
mass_file = fopen("Aircraft.mass");
% part(1): units
fprintf(mass_file, "Lunit" + " " + "=" + " " + "1" + " " + "m\n");
fprintf(mass_file, "Munit" + " " + "=" + " " + "1" + " " + "kg\n");
fprintf(mass_file, "Tunit" + " " + "=" + " " + "1" + " " + "s\n\n");
% part(2): constants
fprintf(mass_file, "g" + " " + "=" + " " + "9.81\n");
fprintf(mass_file, "rho" + " " + "=" + " " + "1.225\n\n");
% part(3): mass & CG
fprintf(mass_file, num2str(MTOW) + " " + num2str(X_cg) + " " + "0" + " " + "0");

fclose(mass_file);

%% Geomtry file
geom_file = fopen("Aircraft.avl",'w');
% part(1): 6 intro lines
fprintf(geom_file,'Aircraft\n');
fprintf(geom_file,"0\n");
fprintf(geom_file,"0" + " " + "0" + " " + "0\n");
fprintf(geom_file, num2str(wing.s) + " " + num2str(wing.MAC) + " " + ...
        num2str(wing.b) + "\n");
fprintf(geom_file, "0" + " " + "0" + " " + "0\n");
fprintf(geom_file, "0\n");

% part(2): surfaces geometry
 % Wing
   fprintf(geom_file, "SURFACE\n");
   fprintf(geom_file, "wing\n");
   fprintf(geom_file, "20" + " " + "1" + " " + "20" + " " + "1\n\n");
   fprintf(geom_file, "YDUPLICATE\n");
   fprintf(geom_file, "0\n\n");
   fprintf(geom_file, "ANGLE\n");
   fprintf(geom_file, "0\n\n");
  % root section
   fprintf(geom_file, "SECTION\n");
   fprintf(geom_file, "0" + " " + "0" + " " + "0" + " " + num2str(wing.Cr) + ...     
          " " + num2str(wing.Root_twist) + "\n\n");
   fprintf(geom_file, "AFILE\n");
   fprintf(geom_file, wing.Airfoil + ".dat\n\n");
  % tip section
   fprintf(geom_file, "SECTION\n");
   fprintf(geom_file, num2str(wing.O_tip) + " " + num2str(wing.b/2) + " " +...
           "0" + " " + num2str(wing.Ct) + " " + num2str(wing.Tip_twist) + "\n\n");
   fprintf(geom_file, "AFILE\n");
   fprintf(geom_file, wing.Airfoil + ".dat\n\n");
  
 % Horizontal tail
   fprintf(geom_file, "SURFACE\n");
   fprintf(geom_file, "Horizontal\n");
   fprintf(geom_file, "20" + " " + "1" + " " + "20" + " " + "1\n\n");
   fprintf(geom_file, "YDUPLICATE\n");
   fprintf(geom_file, "0\n\n");
   fprintf(geom_file, "ANGLE\n");
   fprintf(geom_file, num2str(i_t) + "\n\n");
  % root section
   fprintf(geom_file, "SECTION\n");
   fprintf(geom_file, num2str(htail.L) + " " + "0" + " " + "0" + " " + ...
          num2str(tail.cv) + " " + "0\n\n");
   fprintf(geom_file, "AFILE\n");
   fprintf(geom_file, "NACA 0004.dat\n\n");
   fprintf(geom_file, "CONTROL\n");
   fprintf(geom_file, "elevator" + " " + "1" + " " + num2str(xhinge) + " " + "0" + " " + ...
          "0" + " " + "0" + " " + "1\n\n");
  % tip section
   fprintf(geom_file, "SECTION\n");
   fprintf(geom_file, num2str(htail.L) + " " + num2str(tail.bv/2*cos(tail.DA)) + " " + ...
          num2str(tail.bv/2*sin(tail.DA)) + " " + num2str(htail.C_h) + " " + "0\n\n");
   fprintf(geom_file, "AFILE\n");
   fprintf(geom_file, "NACA 0004.dat\n\n");
   fprintf(geom_file, "CONTROL\n");
   fprintf(geom_file, "elevator" + " " + "1" + " " + num2str(xhinge) + " " + "0" + " " + ...
          "0" + " " + "0" + " " + "1\n\n");
  
fclose(geom_file);
end