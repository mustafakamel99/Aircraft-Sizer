function Figures(wing,htail,tail,dataX,dataY,Alpha_trim_M2,Cm_a_M2,Yle,ccl,cl,v,D2_kg,X_cg)

% Figure 1 (Aircraft Top View Geometry & Airfoil)

figure
subplot(3,1,[1 2])

x_c_w = [0 wing.b/2 wing.b/2 0 -wing.b/2 -wing.b/2];
y_c_w = [0 wing.O_tip (wing.O_tip+wing.Ct) wing.Cr (wing.O_tip+wing.Ct) wing.O_tip];
color = [0.8500 0.3250 0.0980];
patch(x_c_w,y_c_w,color,'HandleVisibility','off')
grid on
title('Aircraft Planform View')
xlabel('y')
ylabel('x')
axis equal
hold on

x_c_t = [0 tail.bv/2 tail.bv/2 0 -tail.bv/2 -tail.bv/2];
y_c_t = [htail.L htail.L (htail.L+tail.cv) (htail.L+tail.cv) (htail.L+tail.cv) htail.L];
color = [0 0.4470 0.7410];
patch(x_c_t,y_c_t,color,'HandleVisibility','off')
hold on
plot(0,X_cg,'*','MarkerSize',10,'MarkerEdgeColor','k')
CG = num2str(round(X_cg*100,2));
legend("C.G is at " + " " + CG + "  cm from LE",'location','best')

subplot(3,1,3)

plot(dataX,dataY,'color','black','linewidth',2)
title(['Airfoil', wing.Airfoil])
xlabel('Chord (m)')
% xlim('auto');
% ylim('auto');
axis equal;
grid on

% Figure 2 (Cm-alpha)

figure
Alpha = [-10,10];
 y1 = Cm_a_M2.*(Alpha-Alpha_trim_M2);
%  y2 = Cm_a_M3.*(Alpha-Alpha_trim_M3);

plot(Alpha,y1,'LineWidth',2)
xlabel('AOA (DEG)')
ylabel('Pitching Moment Coeff. (Cm)') 
legend({'M2','M3'},'location','southeast')
grid on
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
xlim([-5 5])
ylim([-5 5])

% Figure 3 (Wing Distribution)
figure

plot(Yle,ccl,Yle,cl,'LineWidth',2)  
title('Wing Distribution')
xlabel('Spanwise position')
ylabel(' Load Distribution & Local Cl Distribution') 
legend({'Load Distribution','Local Cl Distribution'},'location','northeast')
grid on
axis equal

% Figure 4 (Drag Polar)
figure
plot(v,D2_kg,'LineWidth',2)
xlabel('v(m/s)')
ylabel('Drag(Kg)')
title('Drag polar')
% hold on
% plot(v,D3_kg,'LineWidth',2)
% legend('M2','M3')
grid on
end

