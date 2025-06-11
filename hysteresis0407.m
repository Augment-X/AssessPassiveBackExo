%%

Torque_avg_angle  = load("TORQUE_ANGLE.mat");
Torque_avg_torque = load('TORQUE_FORCE.mat');



    hysteresis_area   = abs(trapz(Torque_avg_angle.TORQUE_whole_x .*(2*pi/360),  Torque_avg_torque.TORQUE_whole_Y)) ;
    hysteresis_area_1 = abs(trapz(Torque_avg_angle.TORQUE_whole_x(1:1020) .*(2*pi/360),  Torque_avg_torque.TORQUE_whole_Y(1:1020))) ;
    hysteresis_area_2 = abs(trapz(Torque_avg_angle.TORQUE_whole_x(1021:end) .*(2*pi/360),  Torque_avg_torque.TORQUE_whole_Y(1021:end))) ;
    hysteresis_area_3 = abs(trapz(flipud(Torque_avg_angle.TORQUE_whole_x(1021:end)) .*(2*pi/360),  flipud(Torque_avg_torque.TORQUE_whole_Y(1021:end)))) 

    hysteresis_percent = hysteresis_area / hysteresis_area_1 * 100;
    hysteresis = vpa(hysteresis_percent,4)  

%%


figure  %%% raw
    t = tiledlayout(1,1,'TileSpacing','Compact');
    h1 = plot(trial_1_load_angle,trial_1_load_torque,'Linestyle','-','Color','#0E606B');
    h1.LineWidth =2;
    hold on;
    h5 = plot(trial_1_unload_angle,trial_1_unload_torque,'Linestyle','--','Color','#0E606B');
    h5.LineWidth =2;
    
    h2 = plot(trial_2_load_angle,trial_2_load_torque,'Linestyle','-','Color','#1597A5');
    h2.LineWidth =2;

    h6 = plot(trial_2_unload_angle,trial_2_unload_torque,'Linestyle','--','Color','#1597A5');
    h6.LineWidth =2;

    h3 = plot(trial_3_load_angle,trial_3_load_torque,'Linestyle','-','Color','#F66F69');
    h3.LineWidth =2;

    h7 = plot(trial_3_unload_angle,trial_3_unload_torque,'Linestyle','--','Color','#F66F69');
    h7.LineWidth =2;
    h4 = plot(Torque_avg_angle.TORQUE_whole_x(1:1020), Torque_avg_torque.TORQUE_whole_Y(1:1020),'-','Color','#FFC24B');
    h4.LineWidth =2;

   
   
    %  fit 
    
    h8 = plot(Torque_avg_angle.TORQUE_whole_x(1021:end), Torque_avg_torque.TORQUE_whole_Y(1021:end),'Linestyle','--','Color','#FFC24B')
    h8.LineWidth =2;


    %
    L2 = legend('1st flexion','1st extension','2nd flexion','2nd extension','3rd flexion','3rd extension','Avg flexion','Avg extension');
    L2.FontSize = 34;
    

    GG = xlabel(t,'Angle (°)'); 
    GG.FontSize = 40;

    GG_1 = ylabel(t,'Torque (N·m)');
    GG_1.FontSize = 40;

    h = title('Angle-torque profile during bending');
    h.FontSize = 40;
    h.FontWeight = "bold" ;
    grid on;
    xlim([-5 90]);
    ylim([-2 42]);

    hysteresis_txt = text(40,2.5,'Avg hysteresis = 56.26%');
    hysteresis_txt.FontSize = 38;