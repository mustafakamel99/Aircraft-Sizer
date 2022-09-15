function [MTOW] = weight_guess_2(pylod,WB_MTOW,mthd)

if mthd == 1 %balsa
       syms x
    eqn=x-1.457*(WB_MTOW+pylod/x)*x- 0.2021==0;
 
MTOW = vpasolve(eqn,x);
MTOW= double(MTOW)
elseif mthd== 2 %foam core composites
       syms x
    eqn =  x-2.1888*(WB_MTOW+pylod/x)*x-0.921==0;
 
MTOW = vpasolve(eqn,x);
MTOW= double(MTOW)

  
elseif mthd ==3 %foam wing
       syms x
    eqn=x -1.3557*(WB_MTOW+pylod/x)*x -0.2453==0;

MTOW = vpasolve(eqn,x);
MTOW= double(MTOW)


elseif mthd == 4 %Molded composites
       syms MTOW
    eqn=x -1.8361*(WB_MTOW+pylod/x)*x-0.0995==0;

MTOW = vpasolve(eqn,x);
MTOW= double(MTOW)
    
else
    error('Please Assign a value between 1 and 4')

end

    

