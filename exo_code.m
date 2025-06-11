clear; close all; clc


addpath('C:\Program Files\Vicon\Nexus2.14\SDK\MATLAB');
addpath('C:\Users\Haibing\OneDrive\OneDrive - Vrije Universiteit Brussel\Desktop\New_folder\new_code\final_measurement_code');
run exo_angle.m ;
run exo_force.m ;  %% dynamic 8
%%

%
% addpath('C:\Program Files\Vicon\Nexus2.14\SDK\MATLAB');
smootgdata_window = 100;
run dimension_list.m ;

%  exo-force / angle

exo_angle_degree_bending_offset = load("exo_angle_degree_bending.mat");
exo_angle_degree_lower_back_joint_offset =load("exo_angle_degree_lower_back_joint.mat");
load("exo_force_chest_offset.mat");
load("exo_force_hip_offset.mat");
load("exo_force_leg_offset.mat");
load("fit_coefficient.mat");  %% fitting force from no-exo condition
no_exo_angle_degree_lower_back_joint = load("no_exo_angle_degree_lower_back_joint.mat");
no_exo_angle_degree_bending = load("no_exo_angle_degree_bending.mat");
load('No_exo_force_chest_offset.mat');
load("No_exo_force_hip_offset.mat");
load("No_exo_force_leg_offset.mat");

%
exo_mass_condition = 0; %%%%%%%%%%%%%%%%%%%%%

   if exo_mass_condition == 0
     exo_upper = 1.1;
     exo_lower = 2.8;
   end

   if exo_mass_condition == 1
     exo_upper = 14.6/9.8;
     exo_lower = 16.6/9.8;
   end

   test_exo_upper = mean(exo_force_chest_offset.chest_verti(offset_a:offset_b)) - mean(No_exo_force_chest_offset.no_exo_chest_verti_offset(offset_a:offset_b));
   test_exo_lower = mean(exo_force_hip_offset.hip_verti(offset_a:offset_b))     - mean(No_exo_force_hip_offset.no_exo_hip_verti_offset(offset_a:offset_b));

   test_exo_upper = 25.5;
   test_exo_lower = 11.37;


%  force_fit condition
fit_condition =  0 ;  %%%%%%%%%%%%%%%%%%%%

  if fit_condition == 0  %%% theory
   
    k_chest_verti_amp  = 1;
    k_chest_verti_hori = 0;

    k_hip_verti_amp  = 1;
    k_hip_verti_hori =  0;

    k_leg_verti_amp  = 1;
    k_leg_verti_hori = 0;
  end

  if fit_condition == 1   %%% real 

    k_chest_verti_amp  = 1;
    k_chest_verti_hori =  10 / 96;

    k_hip_verti_amp  = 84.0840  /  84.8393;
    k_hip_verti_hori =  9.7927 /  84.8393;

    k_leg_verti_amp  = 48.5100  /  46.6206;
    k_leg_verti_hori =  1 /  46.6206;

  end

  if fit_condition == 2  %%% test value

    k_chest_verti_amp  = 1;
    k_chest_verti_hori =  10 / 96;

    k_hip_verti_amp  = 1;
    k_hip_verti_hori =  0;

    k_leg_verti_amp  = 1;
    k_leg_verti_hori =  0;

  end

% angle 

angle_condition = 1;  %%%%%%%%%%%%%%%%%%%%%%%%  0----no offset |||||   1---- offset

  if angle_condition == 0     %%%% no offset

     angle_degree_lower_back_joint  = remove_off(exo_angle_degree_lower_back_joint_offset.angle_degree_lower_back_joint,offset_a,offset_b);
     angle_degree_bending           = remove_off(exo_angle_degree_bending_offset.angle_degree_bending,offset_a,offset_b);
     angle_rad_lower_back_joint = (angle_degree_lower_back_joint)/360*2*pi;
     angle_rad_bending          = (angle_degree_bending)/360*2*pi;

  end

  if angle_condition == 1     %%%%     offset

     angle_degree_lower_back_joint  = (exo_angle_degree_lower_back_joint_offset.angle_degree_lower_back_joint);
     angle_degree_bending           = (exo_angle_degree_bending_offset.angle_degree_bending);
     angle_rad_lower_back_joint = (angle_degree_lower_back_joint)/360*2*pi;
     angle_rad_bending          = (angle_degree_bending)/360*2*pi;

  end

angle_degree_lower_back_joint_offset =  exo_angle_degree_lower_back_joint_offset.angle_degree_lower_back_joint;
angle_degree_bending_offset          =  exo_angle_degree_bending_offset.angle_degree_bending;

angle_degree_lower_back_joint_no_offset =  remove_off(exo_angle_degree_lower_back_joint_offset.angle_degree_lower_back_joint,offset_a,offset_b);
angle_degree_bending_no_offset          =  remove_off(exo_angle_degree_bending_offset.angle_degree_bending,offset_a,offset_b);


%  fitting force from no-exo


% Fit_hip_hori              =  fitting_fun(fit_coefficient(:,1),angle_degree_lower_back_joint);
% Fit_hip_verti             =  fitting_fun(fit_coefficient(:,2),angle_degree_lower_back_joint);
% Fit_hip_hori_top          =  fitting_fun(fit_coefficient(:,3),angle_degree_lower_back_joint);
% Fit_hip_hori_bot          =  fitting_fun(fit_coefficient(:,4),angle_degree_lower_back_joint);
% Fit_hip_verti_front       =  fitting_fun(fit_coefficient(:,5),angle_degree_lower_back_joint);
% Fit_hip_verti_back        =  fitting_fun(fit_coefficient(:,6),angle_degree_lower_back_joint);
% 
% Fit_leg_hori_top          =  fitting_fun(fit_coefficient(:,7),angle_degree_bending);
% Fit_leg_hori_bot          =  fitting_fun(fit_coefficient(:,8),angle_degree_bending);
% Fit_leg_verti             =  fitting_fun(fit_coefficient(:,9),angle_degree_bending);
% Fit_leg_verti_front       =  fitting_fun(fit_coefficient(:,10),angle_degree_bending);
% Fit_leg_verti_back        =  fitting_fun(fit_coefficient(:,11),angle_degree_bending);


Fit_hip_hori              =  fit_hori_force(fit_coefficient(:,1),angle_degree_lower_back_joint);
Fit_hip_verti             =  fit_verti_force(fit_coefficient(:,2),angle_degree_lower_back_joint);
Fit_hip_hori_top          =  fit_hori_force(fit_coefficient(:,3),angle_degree_lower_back_joint);
Fit_hip_hori_bot          =  fit_hori_force(fit_coefficient(:,4),angle_degree_lower_back_joint);
Fit_hip_verti_front       =  fit_verti_force(fit_coefficient(:,5),angle_degree_lower_back_joint);
Fit_hip_verti_back        =  fit_verti_force(fit_coefficient(:,6),angle_degree_lower_back_joint);

Fit_leg_hori_top          =  fit_hori_force(fit_coefficient(:,7),angle_degree_bending);
Fit_leg_hori_bot          =  fit_hori_force(fit_coefficient(:,8),angle_degree_bending);
Fit_leg_verti             =  fit_verti_force(fit_coefficient(:,9),angle_degree_bending);
Fit_leg_verti_front       =  fit_verti_force(fit_coefficient(:,10),angle_degree_bending);
Fit_leg_verti_back        =  fit_verti_force(fit_coefficient(:,11),angle_degree_bending);

% exo_mass_upper  = mean(exo_force_chest_offset.chest_verti(offset_a:offset_b,:)) -  mean(No_exo_force_chest_offset.no_exo_chest_verti_offset);
% exo_mass_lower  = mean(exo_force_hip_offset.hip_verti(offset_a:offset_b,:))     -  mean(Fit_hip_verti(offset_a:offset_b,:)) ;

% Real force my method

no_exo_chest_hori_front_top = mean(No_exo_force_chest_offset.no_exo_chest_hori_front_left_top_offset) + mean(No_exo_force_chest_offset.no_exo_chest_hori_front_right_top_offset);
no_exo_chest_hori_front_bot = mean(No_exo_force_chest_offset.no_exo_chest_hori_front_left_bot_offset) + mean(No_exo_force_chest_offset.no_exo_chest_hori_front_right_bot_offset);
no_exo_chest_hori_back      = mean(No_exo_force_chest_offset.no_exo_chest_hori_back_left_offset) + mean(No_exo_force_chest_offset.no_exo_chest_hori_back_right_offset);
% no_exo_chest_hori           = mean(No_exo_force_chest_offset.no_exo_chest_hori_offset) ;
% no_exo_chest_verti          = mean(No_exo_force_chest_offset.no_exo_chest_verti_offset) ;
no_exo_chest_verti_front    = mean(No_exo_force_chest_offset.no_exo_chest_verti_front_left_offset + No_exo_force_chest_offset.no_exo_chest_verti_front_right_offset);
no_exo_chest_verti_back     = mean(No_exo_force_chest_offset.no_exo_chest_verti_back_left_offset  + No_exo_force_chest_offset.no_exo_chest_verti_back_right_offset);

exo_upper_mass_added_on_chest_verti_front       =  mean(exo_force_chest_offset.chest_verti_front(offset_a:offset_b,:)) - no_exo_chest_verti_front;
exo_upper_mass_added_on_chest_verti_back        =  mean(exo_force_chest_offset.chest_verti_back(offset_a:offset_b,:))  - no_exo_chest_verti_back;
exo_upper_mass_added_on_chest_hori_front_top    =  mean(exo_force_chest_offset.chest_hori_front_top(offset_a:offset_b,:))  - no_exo_chest_hori_front_top;
exo_upper_mass_added_on_chest_hori_front_bot    =  mean(exo_force_chest_offset.chest_hori_front_bot(offset_a:offset_b,:))  - no_exo_chest_hori_front_bot;
exo_upper_mass_added_on_chest_hori_back         =  mean(exo_force_chest_offset.chest_hori_back(offset_a:offset_b,:))  - no_exo_chest_hori_back;

Real_chest_hori_front_top =  exo_force_chest_offset.chest_hori_front_top - ...
                             no_exo_chest_hori_front_top *ones(size(angle_degree_bending)) - ...
                             exo_upper_mass_added_on_chest_hori_front_top;

Real_chest_hori_front_bot =  exo_force_chest_offset.chest_hori_front_bot - ...
                             no_exo_chest_hori_front_bot *ones(size(angle_degree_bending)) - ...
                             exo_upper_mass_added_on_chest_hori_front_bot;

Real_chest_hori_back      =  exo_force_chest_offset.chest_hori_back      - ...
                             no_exo_chest_hori_back      *ones(size(angle_degree_bending)) - ...
                             exo_upper_mass_added_on_chest_hori_back;

Real_chest_hori           =  Real_chest_hori_front_top + Real_chest_hori_front_bot + Real_chest_hori_back;

% Real_chest_verti          =  exo_force_chest_offset.chest_verti    -       no_exo_chest_verti          *ones(size(angle_degree_bending)) - test_exo_upper;

Real_chest_verti_front    =  exo_force_chest_offset.chest_verti_front - ... 
                             no_exo_chest_verti_front * ones(size(angle_degree_bending)) - ...
                             exo_upper_mass_added_on_chest_verti_front ;

Real_chest_verti_back     =  exo_force_chest_offset.chest_verti_back - ...
                             no_exo_chest_verti_back * ones(size(angle_degree_bending)) - ...
                             exo_upper_mass_added_on_chest_verti_back;

Real_chest_verti          =  Real_chest_verti_front  + Real_chest_verti_back ;
%%
exo_lower_mass_added_on_hip_verti_front       =  mean(exo_force_hip_offset.hip_verti_front(offset_a:offset_b,:)) - mean(Fit_hip_verti_front(offset_a:offset_b,:));
exo_lower_mass_added_on_hip_verti_back        =  mean(exo_force_hip_offset.hip_verti_back(offset_a:offset_b,:))  - mean(Fit_hip_verti_back(offset_a:offset_b,:));
exo_lower_mass_added_on_hip_hori_top          =  mean(exo_force_hip_offset.hip_hori_top(offset_a:offset_b,:))    - mean(Fit_hip_hori_top(offset_a:offset_b,:));
exo_lower_mass_added_on_hip_hori_bot          =  mean(exo_force_hip_offset.hip_hori_bot(offset_a:offset_b,:))    - mean(Fit_hip_hori_bot(offset_a:offset_b,:));

% Real_hip_hori_top       =  remove_off((exo_force_hip_offset.hip_hori_top       - Fit_hip_hori_top ),offset_a,offset_b) ;
% Real_hip_hori_bot       =  remove_off((exo_force_hip_offset.hip_hori_bot       - Fit_hip_hori_bot ),offset_a,offset_b) ;

Real_hip_hori_top       =  exo_force_hip_offset.hip_hori_top       - Fit_hip_hori_top    - (exo_lower_mass_added_on_hip_verti_front + exo_lower_mass_added_on_hip_verti_back) * 0.5  .* sin(angle_rad_lower_back_joint) ;
Real_hip_hori_bot       =  exo_force_hip_offset.hip_hori_bot       - Fit_hip_hori_bot    - (exo_lower_mass_added_on_hip_verti_front + exo_lower_mass_added_on_hip_verti_back) * 0.5  .* sin(angle_rad_lower_back_joint) ;

Real_hip_verti_front    =  exo_force_hip_offset.hip_verti_front    - Fit_hip_verti_front - exo_lower_mass_added_on_hip_verti_front  .* cos(angle_rad_lower_back_joint);
Real_hip_verti_back     =  exo_force_hip_offset.hip_verti_back     - Fit_hip_verti_back  - exo_lower_mass_added_on_hip_verti_back   .* cos(angle_rad_lower_back_joint);

Real_hip_hori       =  Real_hip_hori_top    +   Real_hip_hori_bot   ;
Real_hip_verti      =  Real_hip_verti_front +   Real_hip_verti_back ;

exo_lower_mass_added_on_leg_verti_front       =  mean(exo_force_leg_offset.leg_verti_front(offset_a:offset_b,:)) - mean(Fit_leg_verti_front(offset_a:offset_b,:));
exo_lower_mass_added_on_leg_verti_back        =  mean(exo_force_leg_offset.leg_verti_back(offset_a:offset_b,:))  - mean(Fit_leg_verti_back(offset_a:offset_b,:));
exo_lower_mass_added_on_leg_hori_top          =  mean(exo_force_leg_offset.leg_hori_top(offset_a:offset_b,:))    - mean(Fit_leg_hori_top(offset_a:offset_b,:));
exo_lower_mass_added_on_leg_hori_bot          =  mean(exo_force_leg_offset.leg_hori_bot(offset_a:offset_b,:))    - mean(Fit_leg_hori_bot(offset_a:offset_b,:));

Real_leg_hori_top         =  exo_force_leg_offset.leg_hori_top   - Fit_leg_hori_top  -  exo_lower_mass_added_on_leg_hori_top .* sin(angle_rad_bending);
Real_leg_hori_bot         =  exo_force_leg_offset.leg_hori_bot   - Fit_leg_hori_bot  -  exo_lower_mass_added_on_leg_hori_bot .* sin(angle_rad_bending);
Real_leg_verti_front      =  exo_force_leg_offset.leg_verti_front      - Fit_leg_verti_front   - exo_lower_mass_added_on_leg_verti_front .* cos(angle_rad_bending)  ;
Real_leg_verti_back       =  exo_force_leg_offset.leg_verti_back       - Fit_leg_verti_back    - exo_lower_mass_added_on_leg_verti_back  .* cos(angle_rad_bending);

Real_leg_hori             =  Real_leg_hori_top      +    Real_leg_hori_bot   ;
Real_leg_verti            =  Real_leg_verti_front   +    Real_leg_verti_back ;

Fx_chest_offset = Real_chest_hori ;
Fy_chest_offset = Real_chest_verti_front +  Real_chest_verti_back  ;

Fx_hip_offset   = -Real_hip_verti .* sin(angle_rad_lower_back_joint) + Real_hip_hori  .* cos(angle_rad_lower_back_joint) ;
Fy_hip_offset   =  Real_hip_hori  .* sin(angle_rad_lower_back_joint) + Real_hip_verti .* cos(angle_rad_lower_back_joint) ;

Fx_leg_offset   = -Real_leg_verti  .* sin(angle_rad_bending)  + Real_leg_hori .* cos(angle_rad_bending) ;
Fy_leg_offset   =  Real_leg_verti  .* cos(angle_rad_bending)  + Real_leg_hori .* sin(angle_rad_bending);

Fx_offset = Fx_chest_offset + Fx_hip_offset + Fx_leg_offset ;
Fy_offset = Fy_chest_offset + Fy_hip_offset + Fy_leg_offset ;

% Fy1_offset = Fy_chest_offset + Fy_hip_offset + Fy_leg_offset - G_exo;

Torque_chest =  -1 * (OD + OA + AB * cos(angle_rad_lower_back_joint)) .* Real_chest_hori_front_top  - ...
               (OC + OA + AB * cos(angle_rad_lower_back_joint)) .* Real_chest_hori_front_bot  -  ...
               (OE + OA + AB * cos(angle_rad_lower_back_joint)) .* Real_chest_hori_back   + ...
               (d3 - AB * sin(angle_rad_lower_back_joint))      .* Real_chest_verti_front     - ...
               (AB * sin(angle_rad_lower_back_joint) + d3)      .* Real_chest_verti_back           ;


IJ = AJ - AI;

Torque_hip   = -1 * BI * Real_hip_hori_top   -  BJ * Real_hip_hori_bot   +  d4  * Real_hip_verti_front  -  d4  * Real_hip_verti_back ;

% Torque_hip_1   = -1 * IJ /2 * Real_hip_hori_top   +  IJ /2 * Real_hip_hori_bot   +  d4  * Real_hip_verti_front  -  d4  * Real_hip_verti_back ;

Torque_leg   = BK * Real_leg_hori_top   +  BL * Real_leg_hori_bot   +  d7 * Real_leg_verti_front   -  d7  * Real_leg_verti_back  ; 



Torque = BK * Real_leg_hori_top   +  BL * Real_leg_hori_bot   +  d7 * Real_leg_verti_front   -  d7  * Real_leg_verti_back  - ...
         BI * Real_hip_hori_top   -  BJ * Real_hip_hori_bot   +  d4  * Real_hip_verti_front  -  d4  * Real_hip_verti_back  - ...
         (OD + OA + AB * cos(angle_rad_lower_back_joint)) .* Real_chest_hori_front_top  -  (OC + OA + AB * cos(angle_rad_lower_back_joint)) .* Real_chest_hori_front_bot  -  (OE + OA + AB * cos(angle_rad_lower_back_joint)) .* Real_chest_hori_back   + ...
         (d3 - AB * sin(angle_rad_lower_back_joint))      .* Real_chest_verti_front     -  (AB * sin(angle_rad_lower_back_joint) + d3)      .* Real_chest_verti_back           ;

Torque_abs = -1 * Torque;

% dd = (22.5 + 40 + 5.5 + 35)/1000;
% OT = dd;
% TD = OD - dd;
% 
% Torque_by_fixed_beam = -1 * (OD -dd ) .* Real_chest_hori_front_top  - ...
%                       (OC -dd ) .* Real_chest_hori_front_bot  -  ...
%                       (OE -dd ) .* Real_chest_hori_back   + ...
%                       (d3 )      .* Real_chest_verti_front     - ...
%                       (d3 )      .* Real_chest_verti_back           ;

% Torque_chest_T = - (Real_chest_hori) * 98/1000; 
% 
% AAA = Torque_chest + Torque_hip +  Torque_leg;
% 
% hip_anti = AI * Real_hip_hori_top   +  AJ * Real_hip_hori_bot   +  d4  * Real_hip_verti_front  -  d4  * Real_hip_verti_back ;
% 
% % sum = Torque - Torque_by_fixed_beam + MASS_EXO_TORQUE + hip_anti ;
% 
% MASS_EXO_TORQUE =  -1 * test_exo_upper * AB * sin(angle_rad_lower_back_joint) - 1 *  test_exo_lower * BI * sin(angle_rad_lower_back_joint) ;
%%  plot abd hysteresis 

% time =  (1 : length(angle_degree_lower_back_joint_offset)) ./ 100;
% 
% hysterisis_y_1 = angle_degree_bending_offset(1);
% hysterisis_x_1 = 1;
% 
% hysterisis_y_2 = max(angle_degree_bending_offset(1:5500));
% hysterisis_x_2 = find( angle_degree_bending_offset(1:5500) == hysterisis_y_2);
% 
% hysterisis_y_3 = min(angle_degree_bending_offset(5500:8300));
% hysterisis_x_3 = find( angle_degree_bending_offset(5500:8300) == hysterisis_y_3) + 5500;
% 
% hysterisis_y_4 = max(angle_degree_bending_offset(8300:12500));
% hysterisis_x_4 = find( angle_degree_bending_offset(8300:12500) == hysterisis_y_4) + 8300;
% 
% hysterisis_y_5 = min(angle_degree_bending_offset(12500:16500));
% hysterisis_x_5 = find( angle_degree_bending_offset(12500:16500) == hysterisis_y_5) + 12500;
% 
% hysterisis_y_6 = max(angle_degree_bending_offset(16500:21000));
% hysterisis_x_6 = find( angle_degree_bending_offset(16500:21000) == hysterisis_y_6) + 16500;
% 
% hysterisis_y_7 = angle_degree_bending_offset(length(angle_degree_bending_offset));
% hysterisis_x_7 = length(angle_degree_bending_offset);
% 
% color_1 = strings(1,3);
% color = {'red','blue','k'};
% for i = 1:3
% color_1(i) = cell2mat(color(i));
% end 
% 
% fig_1 = figure();
% plt_1 = plot(angle_degree_bending_offset(hysterisis_x_1:hysterisis_x_2),Torque_abs(hysterisis_x_1:hysterisis_x_2));
% plt_1.Color = color_1(1);
% plt_1.LineStyle = '-' ;
% hold on;
% 
% plt_2 = plot(angle_degree_bending_offset(hysterisis_x_2:hysterisis_x_3),Torque_abs(hysterisis_x_2:hysterisis_x_3));
% plt_2.Color = color_1(1);
% plt_2.LineStyle = '--' ;
% 
% plt_3 = plot(angle_degree_bending_offset(hysterisis_x_3:hysterisis_x_4),Torque_abs(hysterisis_x_3:hysterisis_x_4));
% plt_3.Color = color_1(2);
% plt_3.LineStyle = '-' ;
% 
% plt_4 = plot(angle_degree_bending_offset(hysterisis_x_4:hysterisis_x_5),Torque_abs(hysterisis_x_4:hysterisis_x_5));
% plt_4.Color = color_1(2);
% plt_4.LineStyle = '--' ;
% 
% plt_5 = plot(angle_degree_bending_offset(hysterisis_x_5:hysterisis_x_6),Torque_abs(hysterisis_x_5:hysterisis_x_6));
% plt_5.Color = color_1(3);
% plt_5.LineStyle = '-' ;
% 
% plt_6 = plot(angle_degree_bending_offset(hysterisis_x_6:hysterisis_x_7),Torque_abs(hysterisis_x_6:hysterisis_x_7));
% plt_6.Color = color_1(3);
% plt_6.LineStyle = '--' ;
% 
% xlabel('Bending Angle (degree)');
% ylabel('Support Torque (Nm)');
% title('Angle-Torque Relationship')
% grid on;
% axis equal;
% legend('Flexion 1','Extension 1','Flexion 2','Extension 2','Flexion 3','Extension 3');
% xlim([0 80]);
% ylim([-5 25]);
% 
% red_arrow_1 = annotation('arrow',[0.1 0.11],[0.1 0.1]);
% red_arrow_1.Color = color_1(1) ; 
% red_arrow_2 = annotation('arrow',[0.1 0.11],[0.2 0.2]);
% red_arrow_2.Color = color_1(1) ; 
% red_arrow_3 = annotation('arrow',[0.1 0.11],[0.3 0.3]);
% red_arrow_3.Color = color_1(1) ; 
% 
% blue_arrow_1 = annotation('arrow',[0.2 0.21],[0.1 0.1]);
% blue_arrow_1.Color = color_1(2) ; 
% blue_arrow_2 = annotation('arrow',[0.2 0.21],[0.2 0.2]);
% blue_arrow_2.Color = color_1(2) ; 
% blue_arrow_3 = annotation('arrow',[0.2 0.21],[0.3 0.3]);
% blue_arrow_3.Color = color_1(2) ; 
% 
% yellow_arrow_1 = annotation('arrow',[0.3 0.31],[0.1 0.1]);
% yellow_arrow_1.Color = color_1(3) ; 
% yellow_arrow_2 = annotation('arrow',[0.3 0.31],[0.2 0.2]);
% yellow_arrow_2.Color = color_1(3) ; 
% yellow_arrow_3 = annotation('arrow',[0.3 0.31],[0.3 0.3]);
% yellow_arrow_3.Color = color_1(3) ; 
% 
% loading_x_1 = angle_degree_bending_offset(1 : hysterisis_x_2);
% loading_y_1 = Torque_abs(1 : hysterisis_x_2);
% unloading_x_1 = angle_degree_bending_offset(hysterisis_x_2 + 1: hysterisis_x_3);
% unloading_y_1 = Torque_abs(hysterisis_x_2 + 1 : hysterisis_x_3);
% 
% % Define a common x-grid
% x_common = linspace(min([loading_x_1; unloading_x_1]), max([loading_x_1; unloading_x_1]), max(length(loading_x_1), length(unloading_x_1)));
% 
% % Interpolate both curves
% y_loading_interp = interp1(loading_x_1, loading_y_1, x_common, 'linear', 'extrap');
% y_unloading_interp = interp1(unloading_x_1, unloading_y_1, x_common, 'linear', 'extrap');
% 
% % Calculate Hysteresis Area
% hysteresis_area = trapz(x_common, abs(y_loading_interp - y_unloading_interp));
% 
% 
% hysteresis_1_x_closed_up   = [0 angle_degree_bending_offset(hysterisis_x_2)] / 360* 2 * pi;
% hysteresis_1_y_closed_up   = [hysterisis_y_1 hysterisis_y_2];
% hysteresis_1_x_closed_down = [angle_degree_bending_offset(hysterisis_x_3) angle_degree_bending_offset(hysterisis_x_2)] / 360* 2 * pi;
% hysteresis_1_y_closed_down = [hysterisis_y_3 hysterisis_y_2];
% 
% hysteresis_1 = trapz(hysteresis_1_x_closed_up, hysteresis_1_y_closed_up) - trapz(hysteresis_1_x_closed_down, hysteresis_1_y_closed_down);
% 
% hysteresis_1 = trapz(hysteresis_1_x_closed_up, hysteresis_1_y_closed_up) - trapz(hysteresis_1_x_closed_down, hysteresis_1_y_closed_down);
% 
% 
% energy_stored_1   = trapz(hysteresis_1_x_closed_up, hysteresis_1_y_closed_up) ;
% energy_released_1 = trapz(hysteresis_1_x_closed_down, hysteresis_1_y_closed_down);
% 
% % arrow_cordi = load("arrow_cordi.mat");
% % for i = 1:8
% %    x(i) = arrow_cordi.aa(i,1);
% %    y(i) = arrow_cordi.aa(i,2); 
% % end
% % 
% % for i = 1:4
% %     dx = x(2*i)-x(2*i-1) ;
% %     dy = y(2*i)-y(2*i-1) ;
% % 
% % end
% 
% % for i = 1:6
% %    x(i) = aa(7-i,1);
% %    y(i) = aa(7-i,2); 
% % end
% % 
% % for i = 1:3
% %     dx = x(2*i-1)-x(2*i) ;
% %     dy = y(2*i-1)-y(2*i) ;
% % 
% % end
% % 
% %  quiver(x(1), y(1), dx(1), dy(1), 0, 'MaxHeadSize', 1, 'Color', 'r');
% 
% 
% 
% %% misalignment
% 
% load('exo_joint_position.mat');
% load('No_exo_hip_Joint_position.mat');
% y1_L = No_exo_hip_Joint_position(100,1)  * size(exo_joint_position(:,1));
% z1_L = No_exo_hip_Joint_position(100,2)  * size(exo_joint_position(:,1));
% y1_R = No_exo_hip_Joint_position(100,3)  * size(exo_joint_position(:,1));
% z1_R = No_exo_hip_Joint_position(100,4)  * size(exo_joint_position(:,1));
% y2_L = exo_joint_position(:,1);
% z2_L = exo_joint_position(:,2);
% y2_R = exo_joint_position(:,3);
% z2_R = exo_joint_position(:,4);
% 
% misalign_vector_L = [y2_L - y1_L , z2_L - z1_L ] ;
% misalign_vector_R = [y2_R - y1_R , z2_R - z1_R ] ;
% 
% 
% D_misalignment_L = sqrt((misalign_vector_L(:,1) .^2 + misalign_vector_L(:,2) .^2));
% D_misalignment_R = sqrt((misalign_vector_R(:,1) .^2 + misalign_vector_R(:,2) .^2));
% 
% % plot();
% 
% %% exo force no offset------Tom method
% 
% exo_force_chest_offset.chest_hori_front_top_no_offset = remove_off(exo_force_chest_offset.chest_hori_front_top,offset_a,offset_b);
% exo_force_chest_offset.chest_hori_front_bot_no_offset = remove_off(exo_force_chest_offset.chest_hori_front_bot,offset_a,offset_b);
% exo_force_chest_offset.chest_hori_back_no_offset      = remove_off(exo_force_chest_offset.chest_hori_back,offset_a,offset_b);
% exo_force_chest_offset.chest_verti_front_no_offset    = remove_off(exo_force_chest_offset.chest_verti_front,offset_a,offset_b);
% exo_force_chest_offset.chest_verti_back_no_offset     = remove_off(exo_fo  
% exo_force_hip_offset.hip_hori_top_no_offset    = remove_off(exo_force_hip_offset.hip_hori_top,offset_a,offset_b);
% exo_force_hip_offset.hip_hori_bot_no_offset    = remove_off(exo_force_hip_offset.hip_hori_bot,offset_a,offset_b);
% exo_force_hip_offset.hip_verti_front_no_offset = remove_off(exo_force_hip_offset.hip_verti_front,offset_a,offset_b);
% exo_force_hip_offset.hip_verti_back_no_offset  = remove_off(exo_force_hip_offset.hip_verti_back,offset_a,offset_b);
% 
% exo_force_leg_offset.leg_hori_top_no_offset       = remove_off(exo_force_leg_offset.leg_hori_top,offset_a,offset_b);
% exo_force_leg_offset.leg_hori_bot_no_offset       = remove_off(exo_force_leg_offset.leg_hori_bot,offset_a,offset_b);
% exo_force_leg_offset.leg_verti_no_offset          = remove_off(exo_force_leg_offset.leg_verti,offset_a,offset_b);
% exo_force_leg_offset.leg_verti_front_no_offset    = remove_off(exo_force_leg_offset.leg_verti_front,offset_a,offset_b);
% exo_force_leg_offset.leg_verti_back_no_offset     = remove_off(exo_force_leg_offset.leg_verti_back,offset_a,offset_b);
% 
% 
% Fx_chest_exo = exo_force_chest_offset.chest_hori_front_top_no_offset  + ...
%                exo_force_chest_offset.chest_hori_front_bot_no_offset  + ...
%                exo_force_chest_offset.chest_hori_back_no_offset;
% 
% Fy_chest_exo = exo_force_chest_offset.chest_verti_front_no_offset + ...
%                exo_force_chest_offset.chest_verti_back_no_offset  - exo_upper *9.8;
% 
% Fx_hip_exo   = (exo_force_hip_offset.hip_hori_top_no_offset   +  exo_force_hip_offset.hip_hori_bot_no_offset)   .* cos(angle_rad_lower_back_joint) - ...
%                (exo_force_hip_offset.hip_verti_front_no_offset + exo_force_hip_offset.hip_verti_back_no_offset) .* sin(angle_rad_lower_back_joint) - ...
%                 exo_lower * 9.8* sin(angle_rad_lower_back_joint);
% 
% Fy_hip_exo   = (exo_force_hip_offset.hip_hori_top_no_offset    + exo_force_hip_offset.hip_hori_bot_no_offset) .* sin(angle_rad_lower_back_joint) +...
%                (exo_force_hip_offset.hip_verti_front_no_offset + exo_force_hip_offset.hip_verti_back_no_offset) .* cos(angle_rad_lower_back_joint) - ...
%                 exo_lower * 9.8 *cos(angle_rad_lower_back_joint);
% 
% Fx_leg_exo   = -exo_force_leg_offset.leg_verti_no_offset .* sin(angle_rad_bending) +...
%                (exo_force_leg_offset.leg_hori_top_no_offset + exo_force_leg_offset.leg_hori_bot_no_offset) .* cos(angle_rad_bending) ;
% 
% Fy_leg_exo   = exo_force_leg_offset.leg_verti_no_offset  .* cos(angle_rad_bending) +...
%                (exo_force_leg_offset.leg_hori_top_no_offset + exo_force_leg_offset.leg_hori_bot_no_offset) .* sin(angle_rad_bending);
% 
% Fx_exo = Fx_chest_exo + Fx_hip_exo +  Fx_leg_exo ;
% Fy_exo = Fy_chest_exo + Fy_hip_exo +  Fy_leg_exo ;
% 
% % around hip joint
% Torque_chest_exo = (OD + OA + AB * cos(angle_rad_lower_back_joint)) .* exo_force_chest_offset.chest_hori_front_top_no_offset  + ...
%                    (OC + OA + AB * cos(angle_rad_lower_back_joint)) .* exo_force_chest_offset.chest_hori_front_bot_no_offset  + ...
%                    (OE + OA + AB * cos(angle_rad_lower_back_joint)) .* exo_force_chest_offset.chest_hori_back_no_offset  - ...
%                    (d3 - AB      * sin(angle_rad_lower_back_joint)) .* exo_force_chest_offset.chest_verti_front_no_offset  +...
%                    (AB           * sin(angle_rad_lower_back_joint) + d3 - 1/1000) .* exo_force_chest_offset.chest_verti_back_no_offset  ;
% 
% Torque_hip_exo   = BI  * exo_force_hip_offset.hip_hori_top_no_offset       + BJ  * exo_force_hip_offset.hip_hori_bot_no_offset +...
%                    d4 *  exo_force_hip_offset.hip_verti_back_no_offset     - d4  * exo_force_hip_offset.hip_verti_front_no_offset;
% 
% 
% Torque_leg_exo   = -1*BK * exo_force_leg_offset.leg_hori_top_no_offset   + ...
%                    -1*BL * exo_force_leg_offset.leg_hori_bot_no_offset   + ...
%                    d7    * exo_force_leg_offset.leg_verti_back_no_offset - ...
%                    d7    * exo_force_leg_offset.leg_verti_front_no_offset;
% 
% Torque_z_exo     = Torque_chest_exo + Torque_hip_exo +Torque_leg_exo ;
% 
% % mass_torque =  (AB * sin(angle_rad_lower_back_joint)) .* G_exo_matrix;
% % 
% 
% %% exo force and torque fit figure
% 
% Alpha_exo      =  angle_degree_lower_back_joint ;
% Beta_exo       =  angle_degree_bending ;
% 
% 
% 
% angle_Data_exo = [Beta_exo    ,   Beta_exo    ,  Beta_exo    ,   Beta_exo, Beta_exo    ,   Beta_exo    ,  Beta_exo    ,   Beta_exo,Beta_exo    ,   Beta_exo    ,  Beta_exo    ,   Beta_exo];
% 
% force_Data_exo = [Fx_chest_exo,  Fy_chest_exo,  Torque_chest_exo, Fx_hip_exo,  Fy_hip_exo, Torque_hip_exo, Fx_leg_exo, Fy_leg_exo, Torque_leg_exo,  Fx_exo  ,  Fy_exo , Torque_z_exo];
% 
% x_label = {'angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg'};
% y_label = {'Fx-chest','Fy-chest','Torque-chest','Fx-hip', 'Fy-hip', 'Torque-hip','Fx-leg','Fy-leg','Torque-leg','Fx','Fy','Torque-z'};
% 
% figure();
% t = tiledlayout(4,3,'TileSpacing','Compact');
% 
% 
% 
% fit_coefficient_exo =zeros(6,12);
% fit_gof_exo         = zeros(1,12);
% for i = 1:12
%     [xData, yData] = prepareCurveData( angle_Data_exo(:,i), force_Data_exo(:,i) );
%     % Set up fittype and options.
%     ft = fittype( 'poly5' );
%     opts = fitoptions( 'Method', 'LinearLeastSquares' );
%     opts.Robust = 'Bisquare';
%     % Fit model to data.
%    [fitresult, gof] = fit( xData, yData, ft, opts );
%     fitting_force_exo =   fitresult.p1*angle_Data_exo(:,i).^5 + fitresult.p2*angle_Data_exo(:,i).^4 ...
%                         + fitresult.p3*angle_Data_exo(:,i).^3 + fitresult.p4*angle_Data_exo(:,i).^2 + fitresult.p5*angle_Data_exo(:,i).^1 + fitresult.p6 ;
%     fit_coefficient_exo(:,i)    = [fitresult.p1 fitresult.p2 fitresult.p3 fitresult.p4 fitresult.p5 fitresult.p6]'; 
%     Fit_force_exo(:,i)     = fitting_force_exo;
%     fit_gof_exo(:,i)     = gof.adjrsquare;
% %     title('Fitting between force and angle')  
% 
% 
% 
%         nexttile;
%         plot(angle_Data_exo(:,i), force_Data_exo(:,i),'b');
%         hold on;
%         plot(angle_Data_exo(:,i), Fit_force_exo(:,i),'r');
%         grid on;
%         ax = gca;
%         t  = ax.Title;
%         x  = ax.XLabel;
%         y  = ax.YLabel;
% 
%         set(t,"String",y_label{i});
%         set(x,"String",'Angle');       
%         set(y,"String",'Torque');
% 
% 
%         % text(50,10,num2str(fit_gof(i)));
% end
% 
% hold off;
% 
% sgtitle('Force Distribution on the Diverse Segements with Exo')
% %% no exo force no offset
% 
% no_exo_force_chest_offset.chest_hori_front_top_no_offset = remove_off(No_exo_force_chest_offset.no_exo_chest_hori_front_right_top_offset + ...
%                                                                       No_exo_force_chest_offset.no_exo_chest_hori_front_left_top_offset,offset_a,offset_b);
% no_exo_force_chest_offset.chest_hori_front_bot_no_offset = remove_off(No_exo_force_chest_offset.no_exo_chest_hori_front_right_bot_offset + ...
%                                                                       No_exo_force_chest_offset.no_exo_chest_hori_front_left_bot_offset,offset_a,offset_b);
% no_exo_force_chest_offset.chest_hori_back_no_offset      = remove_off(No_exo_force_chest_offset.no_exo_chest_hori_back_left_offset       +...
%                                                                       No_exo_force_chest_offset.no_exo_chest_hori_back_right_offset ,offset_a,offset_b);
% no_exo_force_chest_offset.chest_verti_front_no_offset    = remove_off(No_exo_force_chest_offset.no_exo_chest_verti_front_left_offset     +...
%                                                                       No_exo_force_chest_offset.no_exo_chest_verti_front_right_offset  ,offset_a,offset_b);
% no_exo_force_chest_offset.chest_verti_back_no_offset     = remove_off(No_exo_force_chest_offset.no_exo_chest_verti_back_left_offset     +...
%                                                                       No_exo_force_chest_offset.no_exo_chest_verti_back_right_offset  ,offset_a,offset_b);
% 
% no_exo_force_hip_offset.hip_hori_top_no_offset = remove_off(No_exo_force_hip_offset.no_exo_hip_hori_top_offset,offset_a,offset_b);
% no_exo_force_hip_offset.hip_hori_bot_no_offset = remove_off(No_exo_force_hip_offset.no_exo_hip_hori_bot_offset,offset_a,offset_b);
% no_exo_force_hip_offset.hip_verti_front_no_offset = remove_off(No_exo_force_hip_offset.no_exo_hip_verti_front_offset,offset_a,offset_b);
% no_exo_force_hip_offset.hip_verti_back_no_offset = remove_off(No_exo_force_hip_offset.no_exo_hip_verti_back_offset,offset_a,offset_b);
% 
% no_exo_force_leg_offset.leg_hori_top_no_offset = remove_off(No_exo_force_leg_offset.no_exo_leg_hori_top_offset,offset_a,offset_b);
% no_exo_force_leg_offset.leg_hori_bot_no_offset = remove_off(No_exo_force_leg_offset.no_exo_leg_hori_bot_offset,offset_a,offset_b);
% no_exo_force_leg_offset.leg_verti_no_offset = remove_off(No_exo_force_leg_offset.no_exo_leg_verti_offset,offset_a,offset_b);
% no_exo_force_leg_offset.leg_verti_front_no_offset = remove_off(No_exo_force_leg_offset.no_exo_leg_verti_front,offset_a,offset_b);
% no_exo_force_leg_offset.leg_verti_back_no_offset = remove_off(No_exo_force_leg_offset.no_exo_leg_verti_back,offset_a,offset_b);
% 
% 
% 
% Fx_chest_without_exo = no_exo_force_chest_offset.chest_hori_front_top_no_offset  + ...
%                        no_exo_force_chest_offset.chest_hori_front_bot_no_offset  + ...
%                        no_exo_force_chest_offset.chest_hori_back_no_offset;
% 
% Fy_chest_without_exo = no_exo_force_chest_offset.chest_verti_front_no_offset + ...
%                        no_exo_force_chest_offset.chest_verti_back_no_offset ;
% 
% Fx_hip_without_exo   = (no_exo_force_hip_offset.hip_hori_top_no_offset    + no_exo_force_hip_offset.hip_hori_bot_no_offset)   .*  cos(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi) - ...
%                        (no_exo_force_hip_offset.hip_verti_front_no_offset + no_exo_force_hip_offset.hip_verti_back_no_offset) .*  sin(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi);
% 
% Fy_hip_without_exo   = (no_exo_force_hip_offset.hip_hori_top_no_offset    + no_exo_force_hip_offset.hip_hori_bot_no_offset)   .*  sin(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi) +...
%                        (no_exo_force_hip_offset.hip_verti_front_no_offset + no_exo_force_hip_offset.hip_verti_back_no_offset) .*  cos(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi) ;
% 
% Fx_leg_without_exo   = -no_exo_force_leg_offset.leg_verti_no_offset .* sin(no_exo_angle_degree_bending.angle_degree_bending/360*2*pi) +...
%                        (no_exo_force_leg_offset.leg_hori_top_no_offset + no_exo_force_leg_offset.leg_hori_bot_no_offset) .*  cos(no_exo_angle_degree_bending.angle_degree_bending/360*2*pi) ;
% 
% Fy_leg_without_exo   = no_exo_force_leg_offset.leg_verti_no_offset  .* cos(no_exo_angle_degree_bending.angle_degree_bending/360*2*pi) +...
%                        (no_exo_force_leg_offset.leg_hori_top_no_offset + no_exo_force_leg_offset.leg_hori_bot_no_offset) .*  sin(no_exo_angle_degree_bending.angle_degree_bending/360*2*pi);
% 
% % delta_x  = (Fy_hip + Fy_chest) ./ 100 .*14;
% 
% Fx_without_exo = Fx_chest_without_exo + Fx_hip_without_exo + Fx_leg_without_exo ;
% Fy_without_exo = Fy_chest_without_exo + Fy_hip_without_exo + Fy_leg_without_exo ;
% 
% Torque_chest_without_exo = (OD + OA + AB * cos(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi)) .* no_exo_force_chest_offset.chest_hori_front_top_no_offset  + ...
%                            (OC + OA + AB * cos(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi)) .* no_exo_force_chest_offset.chest_hori_front_bot_no_offset  + ...
%                            (OE + OA + AB * cos(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi)) .* no_exo_force_chest_offset.chest_hori_back_no_offset  - ...
%                            (d3 - AB   * sin(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi)) .* no_exo_force_chest_offset.chest_verti_front_no_offset  +...
%                            (AB   * sin(no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint/360*2*pi) + d3 - 1/1000) .* no_exo_force_chest_offset.chest_verti_back_no_offset  ;
% 
% Torque_hip_without_exo   = BI  * no_exo_force_hip_offset.hip_hori_top_no_offset   + BJ  * no_exo_force_hip_offset.hip_hori_bot_no_offset +...
%                            d4 *  no_exo_force_hip_offset.hip_verti_back_no_offset - d4  * no_exo_force_hip_offset.hip_verti_front_no_offset;
% 
% 
% Torque_leg_without_exo   = -1*BK * no_exo_force_leg_offset.leg_hori_top_no_offset  + ...
%                            -1*BL * no_exo_force_leg_offset.leg_hori_bot_no_offset  + ...
%                            d7    * no_exo_force_leg_offset.leg_verti_back_no_offset - ...
%                            d7    * no_exo_force_leg_offset.leg_verti_front_no_offset;
% 
% Torque_z_without_exo     = Torque_chest_without_exo + Torque_hip_without_exo +Torque_leg_without_exo ;
% 
% %% no exo force and torque fit figure
% 
% Alpha_without_exo       =  no_exo_angle_degree_lower_back_joint.angle_degree_lower_back_joint ;
% Beta_without_exo        =  no_exo_angle_degree_bending.angle_degree_bending ;
% 
% 
% 
% angle_Data_without_exo  = [Beta_without_exo    ,   Beta_without_exo    ,   Beta_without_exo  ,     Beta_without_exo  ,...
%                            Beta_without_exo    ,   Beta_without_exo    ,   Beta_without_exo  ,     Beta_without_exo  ,...
%                            Beta_without_exo    ,   Beta_without_exo    ,   Beta_without_exo  ,     Beta_without_exo  ];
% 
% force_Data_without_exo  = [Fx_chest_without_exo ,   Fy_chest_without_exo ,   Torque_chest_without_exo , Fx_hip_without_exo ,   ...
%                            Fy_hip_without_exo , Torque_hip_without_exo ,   Fx_leg_without_exo ,   Fy_leg_without_exo ,  Torque_leg_without_exo , ...
%                            Fx_without_exo     ,  Fy_without_exo  , Torque_z_without_exo ];
% 
% x_label = {'angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg','angle of leg'};
% y_label = {'Fx-chest','Fy-chest','Torque-chest','Fx-hip', 'Fy-hip', 'Torque-hip','Fx-leg','Fy-leg','Torque-leg','Fx','Fy','Torque-z'};
% 
% figure();
% o = tiledlayout(4,3,'TileSpacing','Compact');
% 
% 
% 
% fit_coefficient_without_exo  =zeros(6,12);
% fit_gof_without_exo          = zeros(1,12);
% 
% for i = 1:12
%     [xData, yData] = prepareCurveData( angle_Data_without_exo (:,i), force_Data_without_exo (:,i) );
%     % Set up fittype and options.
%     ft = fittype( 'poly5' );
%     opts = fitoptions( 'Method', 'LinearLeastSquares' );
%     opts.Robust = 'Bisquare';
%     % Fit model to data.
%    [fitresult, gof] = fit( xData, yData, ft, opts );
%     fitting_force_without_exo  = fitresult.p1*angle_Data_without_exo (:,i).^5 + fitresult.p2*angle_Data_without_exo (:,i).^4 ...
%                     + fitresult.p3*angle_Data_without_exo (:,i).^3 + fitresult.p4*angle_Data_without_exo (:,i).^2 +  fitresult.p5*angle_Data_without_exo (:,i).^1 + fitresult.p6 ;
%     fit_coefficient_without_exo (:,i)    = [fitresult.p1 fitresult.p2 fitresult.p3 fitresult.p4 fitresult.p5  fitresult.p6]'; 
%     Fit_force_without_exo (:,i)     = fitting_force_without_exo ;
%     fit_gof(:,i)     = gof.adjrsquare;
% 
%     title('Fitting between force and angle')  
% 
% 
% 
%         nexttile;
%         plot(angle_Data_without_exo (:,i), force_Data_without_exo (:,i),'b');
%         hold on;
%         plot(angle_Data_without_exo (:,i), Fit_force_without_exo(:,i),'r');
%         grid on;
%         ax = gca;
%         o  = ax.Title;
%         x  = ax.XLabel;
%         y  = ax.YLabel;
% 
%         set(o,"String",y_label{i});
%         set(x,"String",'Angle');       
%         set(y,"String",'Torque');
% 
% 
%         % text(50,10,num2str(fit_gof(i)));
% end
% hold off;
% sgtitle('Force Distribution on the Diverse Segements without exo')
% 
% %% real force 
% 
% Real_chest_hori_front_top_no_offset =  remove_off(Real_chest_hori_front_top,offset_a,offset_b);
% Real_chest_hori_front_bot_no_offset =  remove_off(Real_chest_hori_front_bot,offset_a,offset_b);
% Real_chest_hori_back_no_offset      =  remove_off(Real_chest_hori_back,offset_a,offset_b);
% 
% Real_chest_verti_no_offset_1        =  remove_off(Real_chest_verti,offset_a,offset_b);
% Real_chest_verti_no_offset          =  k_chest_verti_amp * Real_chest_verti_no_offset_1;
% 
% Real_chest_hori_no_offset_1         =  remove_off(Real_chest_hori,offset_a,offset_b);
% 
% Real_chest_hori_no_offset           =  Real_chest_hori_no_offset_1 - Real_chest_verti_no_offset * k_chest_verti_hori;
% 
% Real_hip_verti_no_offset      =  k_hip_verti_amp * remove_off(Real_hip_verti,offset_a,offset_b);
% 
% 
% Real_hip_hori_no_offset       =  remove_off(Real_hip_hori,offset_a,offset_b) - Real_hip_verti_no_offset *  k_hip_verti_hori ;
% 
% Real_leg_verti_no_offset      =  k_leg_verti_amp * remove_off(Real_leg_verti,offset_a,offset_b);
% 
% Real_leg_hori_top_no_offset    =  remove_off(Real_leg_hori_top,offset_a,offset_b);
% Real_leg_hori_bot_no_offset    =  remove_off(Real_leg_hori_bot,offset_a,offset_b);
% Real_leg_hori_no_offset_1      =  Real_leg_hori_bot_no_offset + Real_leg_hori_top_no_offset;
% Real_leg_hori_no_offset        =  Real_leg_hori_no_offset_1 - Real_leg_verti_no_offset *  k_leg_verti_hori ;
% Real_leg_verti_front_no_offset =  remove_off(Real_leg_verti_front,offset_a,offset_b)     ;
% Real_leg_verti_back_no_offset  =  remove_off(Real_leg_verti_back,offset_a,offset_b)      ;
% 
% % Real_hip_hori_top_no_offset    =  remove_off(Real_hip_hori_top,offset_a,offset_b)   - fit   ;
% 
% %% fit _ real _force
% 
% real_fit_Fx_chest       =  fitting_fun(fit_coefficient_exo(:,1),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,1),angle_degree_bending_offset);
% real_fit_Fy_chest       =  fitting_fun(fit_coefficient_exo(:,2),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,2),angle_degree_bending_offset);
% real_fit_Torque_chest       =  fitting_fun(fit_coefficient_exo(:,3),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,3),angle_degree_bending_offset);
% 
% real_fit_Fx_hip         =  fitting_fun(fit_coefficient_exo(:,4),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,4),angle_degree_bending_offset);
% real_fit_Fy_hip         =  fitting_fun(fit_coefficient_exo(:,5),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,5),angle_degree_bending_offset);
% real_fit_Torque_hip     =  fitting_fun(fit_coefficient_exo(:,6),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,6),angle_degree_bending_offset);
% 
% real_fit_Fx_leg       =  fitting_fun(fit_coefficient_exo(:,7),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,7),angle_degree_bending_offset);
% real_fit_Fy_leg       =  fitting_fun(fit_coefficient_exo(:,8),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,8),angle_degree_bending_offset);
% real_fit_Torque_leg   =  fitting_fun(fit_coefficient_exo(:,9),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,9),angle_degree_bending_offset);
% 
% real_fit_Fx           =  fitting_fun(fit_coefficient_exo(:,10),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,10),angle_degree_bending_offset);
% real_fit_Fy           =  fitting_fun(fit_coefficient_exo(:,11),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,11),angle_degree_bending_offset);
% real_fit_Torque       =  fitting_fun(fit_coefficient_exo(:,12),angle_degree_bending_offset) - fitting_fun(fit_coefficient_without_exo(:,12),angle_degree_bending_offset)  ;
% 
% real_fit = [real_fit_Fx_chest  real_fit_Fy_chest real_fit_Torque_chest...
%             real_fit_Fx_hip    real_fit_Fy_hip   real_fit_Torque_hip  ...
%             real_fit_Fx_leg    real_fit_Fy_leg   real_fit_Torque_leg ...
%             real_fit_Fx        real_fit_Fy       real_fit_Torque];
% figure();
% p = tiledlayout(4,3,'TileSpacing','Compact');
%        for i =  1:12
%         nexttile;
%         plot(angle_degree_bending_offset , real_fit (:,i),'b');
%         hold on;
% 
%         grid on;
%         ax = gca;
%         o  = ax.Title;
%         x  = ax.XLabel;
%         y  = ax.YLabel;
% 
%         set(o,"String",y_label{i});
%         set(x,"String",'Angle');       
%         set(y,"String",'Torque');
% 
% 
%         % text(50,10,num2str(fit_gof(i)));
%        end
% hold off;
% sgtitle('Force Distribution on the Diverse Segements without exo')
% %% 
% 
% Real_hip_hori_top_no_offset       =  remove_off(exo_force_hip_offset.hip_hori_top,offset_a,offset_b) ;
% 
% Real_hip_hori_bot_no_offset       =  remove_off(exo_force_hip_offset.hip_hori_bot,offset_a,offset_b) ;
% 
% %% parameters prepare for dynamic figures
% 
% 
% 
% 
% 
% %% figure
% time =  (1:length(Real_chest_hori_front_top))/100;
% 
% figure(); 
% t = tiledlayout(3,1,'TileSpacing','Compact');
% 
% ax1 = nexttile;
% plot(time,Real_chest_hori_front_top);grid on; title('Force Distribution on the Chest'); 
% % xlabel('Time(s)'); ylabel('Force(N)');
% hold on; 
% plot(time,Real_chest_hori_front_bot); 
% plot(time,Real_chest_hori_back); 
% plot(time,Real_chest_verti);    
% legend('Chest Hori Front Top','Chest Hori Front Bot','Chest Hori Back','Chest Verti');
% hold off;
% 
% ax2 = nexttile;
% plot(time,Real_hip_hori);grid on;       
% hold on; 
% plot(time,Real_hip_verti);      
% title('Force Distribution on the Hip'); 
% % xlabel('Time(s)'); ylabel('Force(N)');
% legend('Hip Hori','Hip Verti');
% hold off;
% 
% ax3 = nexttile;
% plot(time,Real_leg_hori_top);grid on;   
% hold on; 
% plot(time,Real_leg_hori_bot);   
% plot(time,Real_leg_verti); 
% title('Force Distribution on the Legs'); 
% % xlabel('Time(s)'); ylabel('Force(N)');
% legend('Leg Hori Top','Leg Hori Bot','Leg Verti');
% hold off;
% 
% sgtitle(t,'Force Distribution on the Diverse Segements')
% 
% xlabel(t,'Time(s)'); ylabel(t,'Force(N)');
% linkaxes([ax1 ax2 ax3],'x');
% ax1.XLim = [0 max(time)+0.2];
% 
% %% force analysis
% 
% 
% 
% 
% 
% % Real_hip_hori_no_offset = Real_hip_hori_no_offset - Real_hip_verti_no_offset .* 0.65;
% 
% 
% G_exo = (exo_upper + exo_lower)*9.8 ;
% G_exo_matrix = G_exo * ones(size(angle_degree_bending));
% 
% Fx_chest = Real_chest_hori_no_offset ;
% Fy_chest = Real_chest_verti_no_offset  ;
% 
% % hip_cos_angle_matrix = zeros(size(angle_rad_lower_back_joint));
% % hip_sin_angle_matrix = zeros(size(angle_rad_lower_back_joint));
% % 
% % for i  = 1:length(angle_rad_lower_back_joint)
% % 
% %     if angle_rad_lower_back_joint(i) >= 0
% % 
% %         hip_cos_angle_matrix(i) = cos(angle_rad_lower_back_joint(i));
% %         hip_sin_angle_matrix(i) = sin(angle_rad_lower_back_joint(i));
% % 
% %     end
% %     if angle_rad_lower_back_joint(i) < 0
% % 
% %         hip_cos_angle_matrix(i) = -cos(angle_rad_lower_back_joint(i));
% %         hip_sin_angle_matrix(i) = -sin(angle_rad_lower_back_joint(i));
% % 
% %     end
% % end
% 
% Fx_hip   = Real_hip_hori_no_offset .* cos(angle_rad_lower_back_joint) - Real_hip_verti_no_offset .* sin(angle_rad_lower_back_joint);
% Fy_hip   = Real_hip_hori_no_offset .* sin(angle_rad_lower_back_joint) + Real_hip_verti_no_offset .* cos(angle_rad_lower_back_joint) ;
% 
% Fx_leg   = -Real_leg_verti_no_offset .* sin(angle_rad_bending) + (Real_leg_hori_no_offset) .* cos(angle_rad_bending) ;
% Fy_leg   = Real_leg_verti_no_offset  .* cos(angle_rad_bending) + (Real_leg_hori_no_offset) .* sin(angle_rad_bending);
% 
% % delta_x  = (Fy_hip + Fy_chest) ./ 100 .*14;
% 
% Fx = Fx_chest + Fx_hip + Fx_leg ;
% Fy = Fy_chest + Fy_hip + Fy_leg ;
% 
% Fy1 = Fy_chest + Fy_hip + Fy_leg - G_exo;
% 
% 
% fxx = -67 + 110*cos(16.5/360*2*pi) - 171 *sin(16.5/360*2*pi) + 27 *sin(68/360*2*pi) - 140 *cos(68/360*2*pi);
% 
% %% torque around hip joint (point B)
% 
% 
% % 
% % mass_torque =  (AB * sin(angle_rad_lower_back_joint)) .* G_exo_matrix;
% % 
% 
% 
% 
% %% Torque 
% 



%% FUNCTION

function [C] = fitting_fun(A,B)         %% A fitting coefficient got from no-exo code
  C = A(1)*B.^5 + A(2)*B.^4 + A(3)*B.^3 + A(4)*B.^2 + A(5)*B.^1+ A(6)*B .^0 ;  %% B angle got from exo condition
    
end

function [B] = remove_off(A,offset_a,offset_b)
  offset     = sum(A(offset_a:offset_b,:))/(offset_b-offset_a+1);
  samples    = ones(size(A));
  offsetMat  = samples .* offset ;
  B          = A - offsetMat ;
end


function [C] = fit_hori_force(A,B)     %% A = PARAMETER , B =  ANGLE
   
    C = A(1) * sin(B/360*2*pi) + A(2) ;
end

function [C] = fit_verti_force(A,B)
   
    C = A(1) * cos(B/360*2*pi) + A(2) ;

end