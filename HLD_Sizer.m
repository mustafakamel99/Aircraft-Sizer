function [flaps,flap,CL_Max_flappped] = HLD_Sizer(CLmax,cL_max_desired,wing)
Delta_CL_Max=abs(cL_max_desired-CLmax);
Delta_Cl_Max_flap=[0.9 1.3 1.6];

if Delta_CL_Max < 0.1
    flaps= 'no flaps needed';
    CL_Max_flappped=CLmax;
    flap.b=0;
    flap.c=0;
    flap.type=0;
else
    flaps= 'flaps needed';
    CL_Max_flappped=cL_max_desired;
    Area_Ratio=Delta_CL_Max./Delta_Cl_Max_flap;
    
    if Area_Ratio(1)<=0.5
        S_flapped=Area_Ratio(1)*wing.s;
        flap.b=S_flapped/(2*wing.Cr);
        flap.c=0.3*wing.Cr;
        flap.type= 'Plain';
    elseif Area_Ratio(2)<=0.5
        S_flapped=Area_Ratio(2)*wing.s;
        flap.b=S_flapped/(2*wing.Cr);
        flap.c=0.3*wing.Cr;
        flap.type= 'Single slotted';
    else
        S_flapped=Area_Ratio(3)*wing.s;
        flap.b=S_flapped/(2*wing.Cr);
        flap.c=0.3*wing.Cr;
        flap.type= 'Double slotted';
    end
end
end