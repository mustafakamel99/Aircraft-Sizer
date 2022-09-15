%{
Weight guess funcion uses the data from reports of previous years to
estimate the weight of the aircraft depending on how much usefull weight
it will carry. Usefull weight = Payload+baterry. the function receives
three
inputs the first one is the payload weight and the second one is the
battery weight while the third one is the manufacturing technique of the
wing. the function will then use the linear fit function gathered from
the data of the reports of
the previous competitions and estimate the maximum take of weight of the
aircraft.

Manufacturing Technique :

Balsa = 1
Foam Core Composites = 2
Foam Hotwire = 3
Molded Composites = 4

%}

function [MTOW] = weight_guess(pylod,bat_w,mthd)

usfl = pylod+bat_w;


if mthd == 1 %balsa
    MTOW = 1.457*usfl + 0.2021;
  
elseif mthd== 2 %foam core composites
    MTOW =  2.1888*usfl-0.921;
  
elseif mthd ==3 %foam wing
    MTOW = 1.3557*usfl +0.2453;

elseif mthd == 4 %Molded composites
    MTOW = 1.8361*usfl+ 0.0995;
    
else
    error('Please Assign a value between 1 and 4')

end
