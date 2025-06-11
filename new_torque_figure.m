


Pos_No_exo_lower_back_Joint = load('No_exo_lower_back_Joint_position.mat');
Pos_No_exo_lower_back_Joint = Pos_No_exo_lower_back_Joint.No_exo_lower_back_Joint_position ;
Pos_A_static_left     = [Pos_No_exo_lower_back_Joint(:,1) Pos_No_exo_lower_back_Joint(:,2)];
Pos_A_static_right    = [Pos_No_exo_lower_back_Joint(:,3) Pos_No_exo_lower_back_Joint(:,4)];
Pos_A_static          = (Pos_A_static_left + Pos_A_static_right) /2;
Pos_A_dynamic         = ones(size(angle_rad_lower_back_joint)) * mean(Pos_A_static);
Pos_A                 = Pos_A_dynamic /1000 ;
Pos_O          = [Pos_A(:,1)  Pos_A(:,2) + OA];

Pos_P_leg_L = [marker_29(:,2) marker_29(:,3)]/1000;  %% (y , z)

Pos_P_leg_R = [marker_34(:,2) marker_34(:,3)]/1000;  %% (y , z)

Pos_B     = [Pos_A(:,1)-AB*sin(angle_rad_lower_back_joint) Pos_A(:,2)-AB*cos(angle_rad_lower_back_joint)];

Vec_B_P_leg_L = Pos_P_leg_L - Pos_B;
Vec_B_P_leg_R = Pos_P_leg_R - Pos_B;
%% FORCE

F_N_TOP_L = exo_force_data_offset(:,4) - Fit_leg_hori_top/2 ;
F_N_TOP_R = exo_force_data_offset(:,22) - Fit_leg_hori_top/2 ;
F_N_BOT_L = (exo_force_data_offset(:,3) + exo_force_data_offset(:,5) -Fit_leg_hori_bot/2);
F_N_BOT_R = exo_force_data_offset(:,21) + exo_force_data_offset(:,23) -Fit_leg_hori_bot/2;

F_T_FRONT_L = exo_force_data_offset(:,2) - Fit_leg_verti_front/2;
F_T_FRONT_R = exo_force_data_offset(:,19) - Fit_leg_verti_front/2;
F_T_BACK_L  = exo_force_data_offset(:,1) - Fit_leg_verti_back/2;
F_T_BACK_R  = exo_force_data_offset(:,20) - Fit_leg_verti_back/2;

F_F10_L = [F_N_TOP_L.*cos(angle_rad_bending), F_N_TOP_L.*sin(angle_rad_bending).*-1] ;
F_F11_L = [F_N_BOT_L.*cos(angle_rad_bending), F_N_BOT_L.*sin(angle_rad_bending).*-1]  ;
F_F12_L = [F_T_BACK_L.*sin(angle_rad_bending).*-1, F_T_BACK_L.*cos(angle_rad_bending).*-1] ;
F_F13_L = [F_T_FRONT_L.*sin(angle_rad_bending).*-1, F_T_FRONT_L.*cos(angle_rad_bending).*-1];

F_F10_R = [F_N_TOP_R.*cos(angle_rad_bending), F_N_TOP_R.*sin(angle_rad_bending).*-1] ;
F_F11_R =  [F_N_BOT_R.*cos(angle_rad_bending), F_N_BOT_R.*sin(angle_rad_bending).*-1]  ;
F_F12_R = [F_T_BACK_R.*sin(angle_rad_bending).*-1, F_T_BACK_R.*cos(angle_rad_bending).*-1] ;
F_F13_R = [F_T_FRONT_R.*sin(angle_rad_bending).*-1, F_T_FRONT_R.*cos(angle_rad_bending).*-1];

% F_F12_L = [0.*cos(angle_rad_bending) 0.*cos(angle_rad_bending)] ;
% F_F13_L = [0.*cos(angle_rad_bending) 0.*cos(angle_rad_bending)] ;
% F_F12_R = [0.*cos(angle_rad_bending) 0.*cos(angle_rad_bending)] ;
% F_F13_R = [0.*cos(angle_rad_bending) 0.*cos(angle_rad_bending)] ;

%% TORQUE

F_leg_x_L       = F_F10_L(:,1) + F_F11_L(:,1) + F_F12_L(:,1) + F_F13_L(:,1);
F_leg_y_L       = F_F10_L(:,2) + F_F11_L(:,2) + F_F12_L(:,2) + F_F13_L(:,2);
F_leg_L         = [F_leg_x_L  F_leg_y_L];
% T_around_B_L = Vec_B_P_leg_L(:,1) .* F_leg_L(:,2) - Vec_B_P_leg_L(:,2) .* F_leg_L(:,1);
T_around_B_L = Vec_B_P_leg_L(1,1) .* F_leg_L(:,2) - Vec_B_P_leg_L(1,2) .* F_leg_L(:,1);


F_leg_x_R       = F_F10_R(:,1) + F_F11_R(:,1) + F_F12_R(:,1) + F_F13_R(:,1);
F_leg_y_R       = F_F10_R(:,2) + F_F11_R(:,2) + F_F12_R(:,2) + F_F13_R(:,2);
F_leg_R         = [F_leg_x_R  F_leg_y_R]; 
% T_around_B_R = Vec_B_P_leg_R(:,1) .* F_leg_R(:,2) - Vec_B_P_leg_R(:,2) .* F_leg_R(:,1);
T_around_B_R = Vec_B_P_leg_R(1,1) .* F_leg_R(:,2) - Vec_B_P_leg_R(1,2) .* F_leg_R(:,1);



T_around_B = (T_around_B_L + T_around_B_R );

T_around_B_SM = remove_off(T_around_B,offset_a,offset_b);
%% trial 1
    
    % point 
    cursor_point   = [1 1500 3000 5000 6000 8000 10000 length(angle_degree_right_leg)];
    trial_1_CP_y   = max(T_around_B_SM(cursor_point(2):cursor_point(3)))           ; %% load-unload critical point
    trial_1_CP_x   = cursor_point(2) -1 + find(  T_around_B_SM(cursor_point(2):cursor_point(3))  == trial_1_CP_y) ;
    trial_1_end_y  = min(T_around_B_SM(cursor_point(3):cursor_point(4))) ;
    trial_1_end_x  = cursor_point(3) -1 + find( T_around_B_SM(cursor_point(3):cursor_point(4)) ==  trial_1_end_y ) ;

    trial_2_CP_y   = max(T_around_B_SM(cursor_point(4):cursor_point(5)))           ; %% load-unload critical point
    trial_2_CP_x   = cursor_point(4) -1 + find(  T_around_B_SM(cursor_point(4):cursor_point(5))  == trial_2_CP_y) ;
    trial_2_end_y  = min(T_around_B_SM(cursor_point(5):cursor_point(6))) ;
    trial_2_end_x  = cursor_point(5) -1 + find( T_around_B_SM(cursor_point(5):cursor_point(6)) ==  trial_2_end_y ) ;

    trial_3_CP_y   = max(T_around_B_SM(cursor_point(6):cursor_point(7)))           ; %% load-unload critical point
    trial_3_CP_x   = cursor_point(6) -1 + find(  T_around_B_SM(cursor_point(6):cursor_point(7))  == trial_3_CP_y) ;
    trial_3_end_y  = min(T_around_B_SM(cursor_point(7):cursor_point(8))) ;
    trial_3_end_x  = cursor_point(7) -1 + find( T_around_B_SM(cursor_point(7):cursor_point(8)) ==  trial_3_end_y ) ;
    
    % angle-toruqe,  devided into load/unload phases for 3 trials 
    trial_1_load_angle    = angle_degree_bending(1:trial_1_CP_x);
    trial_1_load_torque   =  T_around_B_SM(1:trial_1_CP_x);
    trial_1_unload_angle  = angle_degree_bending(trial_1_CP_x +1 : trial_1_end_x);
    trial_1_unload_torque = T_around_B_SM(trial_1_CP_x +1 : trial_1_end_x);
    [trial_1_load_angle_uni,ia,~] = unique(trial_1_load_angle);
    [trial_1_load_angle_sort, idx] = sort(trial_1_load_angle_uni);
    [trial_1_load_torque_uni,ia,~] = unique(trial_1_load_torque);
    [trial_1_load_torque_sort, idx] = sort(trial_1_load_torque);
    [trial_1_unload_angle_uni,ia,~] = unique(trial_1_unload_angle);
    [trial_1_unload_angle_sort, idx] = sort(trial_1_unload_angle);
    [trial_1_unload_torque_uni,ia,~] = unique(trial_1_unload_torque);
    [trial_1_unload_torque_sort, idx] = sort(trial_1_unload_torque);
 
    trial_2_load_angle    = angle_degree_bending(trial_1_end_x + 1:trial_2_CP_x);
    trial_2_load_torque   =  T_around_B_SM(trial_1_end_x + 1:trial_2_CP_x);
    trial_2_unload_angle  = angle_degree_bending(trial_2_CP_x +1 : trial_2_end_x);
    trial_2_unload_torque = T_around_B_SM(trial_2_CP_x +1 : trial_2_end_x);
    [trial_2_load_angle_uni,ia,~] = unique(trial_2_load_angle);
    [trial_2_load_angle_sort, idx] = sort(trial_2_load_angle);
    [trial_2_load_torque_uni,ia,~] = unique(trial_2_load_torque);
    [trial_2_load_torque_sort, idx] = sort(trial_2_load_torque);
    [trial_2_unload_angle_uni,ia,~] = unique(trial_2_unload_angle);
    [trial_2_unload_angle_sort, idx] = sort(trial_2_unload_angle);
    [trial_2_unload_torque_uni,ia,~] = unique(trial_2_unload_torque);
    [trial_2_unload_torque_sort, idx] = sort(trial_2_unload_torque);

    trial_3_load_angle    = angle_degree_bending(trial_2_end_x + 1:trial_3_CP_x);
    trial_3_load_torque   =  T_around_B_SM(trial_2_end_x + 1:trial_3_CP_x);
    trial_3_unload_angle  = angle_degree_bending(trial_3_CP_x +1 : trial_3_end_x);
    trial_3_unload_torque = T_around_B_SM(trial_3_CP_x +1 : trial_3_end_x);
    [trial_3_load_angle_uni,ia,~] = unique(trial_3_load_angle);
    [trial_3_load_angle_sort, idx] = sort(trial_3_load_angle);
    [trial_3_load_torque_uni,ia,~] = unique(trial_3_load_torque);
    [trial_3_load_torque_sort, idx] = sort(trial_3_load_torque);
    [trial_3_unload_angle_uni,ia,~] = unique(trial_3_unload_angle);
    [trial_3_unload_angle_sort, idx] = sort(trial_3_unload_angle);
    [trial_3_unload_torque_uni,ia,~] = unique(trial_3_unload_torque);
    [trial_3_unload_torque_sort, idx] = sort(trial_3_unload_torque);

    % interpl 

    three_torque_matrix = [trial_1_CP_y,trial_2_CP_y,trial_3_CP_y];
    % max_torque_value  = max(max_torque_value);
    avg_torque_value  = mean(three_torque_matrix);
   
    fit_load_torque  = linspace(0,avg_torque_value,10000);
    fit_load_angle_1 = interp1(trial_1_load_torque_sort, trial_1_load_angle_sort, fit_load_torque, 'linear');
    fit_load_angle_2 = interp1(trial_2_load_torque_sort, trial_2_load_angle_sort, fit_load_torque, 'linear');
    fit_load_angle_3 = interp1(trial_3_load_torque_sort, trial_3_load_angle_sort, fit_load_torque, 'linear');

    fit_load_angle = zeros(size(fit_load_torque)) ;
    for i = 1:length(fit_load_torque)
        if fit_load_torque(i) < three_torque_matrix(3)
           fit_load_angle(i) =  ( fit_load_angle_1(i) + fit_load_angle_2(i) + fit_load_angle_3(i)  ) ./3 ;
        
    elseif fit_load_torque(i) >= three_torque_matrix(3)
             fit_load_angle(i) =  (  fit_load_angle_2(i) + fit_load_angle_3(i)  ) ./2 ;
             % fit_angle(i) =   fit_angle_2(i) ;
    else 
             fit_load_angle(i) =   fit_load_angle_2(i) ;

        end
    end
    
    fit_x = fit_load_angle;
    fit_y = fit_load_torque;
    [xData, yData] = prepareCurveData( fit_x, fit_y);

    % Set up fittype and options.
    ft = fittype( 'gompertz' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    opts.StartPoint = [40.4295581921676 0.0388603523819543 24.9212751049772 -0.0164888619816812];
    opts.Lower = [-Inf -Inf -Inf 0];
    % Fit model to data.
    [fitresult, gof] = fit( xData, yData, ft, opts );
    k_fit_load       = [fitresult.a fitresult.b fitresult.c fitresult.d];

    max_angle_3_trials = [max(angle_degree_bending(cursor_point(2):cursor_point(3))) max(angle_degree_bending(cursor_point(4):cursor_point(5))) max(angle_degree_bending(cursor_point(6):cursor_point(7)))];
    
    final_load_angle  = 0:0.0001:mean(max_angle_3_trials) ;
    final_load_torque = fit_fun_gom(final_load_angle,k_fit_load);
     
    max_angle_for_icc = 0:0.0001:max(max_angle_3_trials) ;
    trial_1_fit_load_torque = fit_fun_gom(max_angle_for_icc,fit_gom(fit_load_angle_1,fit_load_torque));
    trial_2_fit_load_torque = fit_fun_gom(max_angle_for_icc,fit_gom(fit_load_angle_2,fit_load_torque));
    trial_3_fit_load_torque = fit_fun_gom(max_angle_for_icc,fit_gom(fit_load_angle_3,fit_load_torque));

% figure
  % figure
  % plot(final_load_angle,trial_1_fit_load_torque,'r'); hold on
  % plot(final_load_angle,trial_2_fit_load_torque,'r')
  % plot(final_load_angle,trial_3_fit_load_torque,'r')
  % plot(trial_1_load_angle,trial_1_load_torque,'b')


function [C] = fit_fun_gom(A,B)  %% A = X ; B = COEFFI
    
         
          C = B(4) + (B(1)-B(4)).*exp(-exp(-B(2).*(A-B(3))));
end


function [C] =  fit_gom(A,B)
     
    [xData, yData] = prepareCurveData( A, B);
    % Set up fittype and options.
    ft = fittype( 'gompertz' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    % Fit model to data.
    [fitresult, gof] = fit( xData, yData, ft, opts );
    C       = [fitresult.a fitresult.b fitresult.c fitresult.d];
end
%%
    % figure()
    % 
    % 
    % plot(angle_degree_bending, T_around_B_SM,'--')
    % hold on
    % plot(fit_angle,fit_torque);
    % hold off

    fit_unload_torque  = linspace(avg_torque_value,0,10000);
    fit_unload_angle_1 = interp1(trial_1_unload_torque_sort, trial_1_unload_angle_sort, fit_unload_torque, 'linear');
    fit_unload_angle_2 = interp1(trial_2_unload_torque_sort, trial_2_unload_angle_sort, fit_unload_torque, 'linear');
    fit_unload_angle_3 = interp1(trial_3_unload_torque_sort, trial_3_unload_angle_sort, fit_unload_torque, 'linear');

    fit_unload_angle = zeros(size(fit_unload_torque)) ;
    for i = 1:length(fit_unload_torque)
        if fit_unload_torque(i) < three_torque_matrix(3)
           fit_unload_angle(i) =  ( fit_unload_angle_1(i) + fit_unload_angle_2(i) + fit_unload_angle_3(i)  ) ./3 ;
        
    elseif fit_unload_torque(i) >= three_torque_matrix(3)
             fit_unload_angle(i) =    fit_unload_angle_2(i) .* (mean(max_angle_3_trials)/max_angle_3_trials(2));
    else 
             fit_unload_angle(i) =   fit_unload_angle_2(i) ;
    

        end
    end
    
    fit_x = fit_unload_angle;
    fit_y = fit_unload_torque;
    [xData, yData] = prepareCurveData( fit_x, fit_y);

    % Set up fittype and options.
    ft = fittype( 'gauss3' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
    opts.StartPoint = [40.4281945501619 85.0786938457495 2.71919814897331 19.6906871027017 79.5291860103172 5.24170705113465 16.8451864324368 68.743188569819 6.73880428864232];

    % Fit model to data.
    [fitresult, gof] = fit( xData, yData, ft, opts );

    k_fit_unload   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];

    final_unload_angle  = mean(max_angle_3_trials):-0.0001:0 ;
    final_unload_torque = fit_fun_gauss(final_unload_angle,k_fit_unload);

    max_angle_for_icc_1 = max(max_angle_3_trials):-0.0001:0 ;
    trial_1_fit_unload_torque = fit_fun_gauss(max_angle_for_icc_1 ,fit_gauss(fit_unload_angle_1,fit_unload_torque));
    trial_2_fit_unload_torque = fit_fun_gauss(max_angle_for_icc_1 ,fit_gauss(fit_unload_angle_2,fit_unload_torque));
    trial_3_fit_unload_torque = fit_fun_gauss(max_angle_for_icc_1 ,fit_gauss(fit_unload_angle_3,fit_unload_torque));

    function [C] =  fit_gauss(A,B)
     
    [xData, yData] = prepareCurveData( A, B);
    % Set up fittype and options.
    ft = fittype( 'gauss3' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    % Fit model to data.
    [fitresult, gof] = fit( xData, yData, ft, opts );
    C       = [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
    end

    % figure
  % figure
  % plot(final_unload_angle,trial_1_fit_unload_torque,'r'); hold on
  % plot(final_unload_angle,trial_2_fit_unload_torque,'r')
  % plot(final_unload_angle,trial_3_fit_unload_torque,'r')
  % plot(trial_1_unload_angle,trial_1_unload_torque,'b')
  %

    %% double fit
    

    loading_angle1 = 0:0.0001:57;

    the_2nd_fit_trial_1_load_angle = trial_1_load_angle(trial_1_load_angle < 57);
    
    up_cp_1 = max(find(trial_1_load_angle < 57)) ;
    up_cp_2 = max(find(trial_2_load_angle < 57)) ;
    up_cp_3 = max(find(trial_3_load_angle < 57)) ;


    k_2nd_fit_trial_1_load = fit_gauss_3(trial_1_load_angle(1:up_cp_1),trial_1_load_torque(1:up_cp_1));
    k_2nd_fit_trial_2_load = fit_gauss_3(trial_2_load_angle(1:up_cp_2),trial_2_load_torque(1:up_cp_2));
    k_2nd_fit_trial_3_load = fit_gauss_3(trial_3_load_angle(1:up_cp_3),trial_3_load_torque(1:up_cp_3));


    fit_2nd_trial_1_load = fit_fun_gauss(trial_1_load_angle(1:up_cp_1),k_2nd_fit_trial_1_load);
    fit_2nd_trial_2_load = fit_fun_gauss(trial_2_load_angle(1:up_cp_2),k_2nd_fit_trial_2_load);
    fit_2nd_trial_3_load = fit_fun_gauss(trial_3_load_angle(1:up_cp_3),k_2nd_fit_trial_3_load);
    
    fit_2nd_load_cp1 = (fit_fun_gauss(loading_angle1,k_2nd_fit_trial_1_load) + ...
                        fit_fun_gauss(loading_angle1,k_2nd_fit_trial_2_load) + ...
                        fit_fun_gauss(loading_angle1,k_2nd_fit_trial_3_load)) ./3 ;
    
    rows = 1:550000;
    rows_1 = 550001: length(final_load_torque) ;
    new_load = zeros(size(final_load_torque));

    new_load(rows)   = fit_2nd_load_cp1(rows);
    new_load(rows_1) = final_load_torque(rows_1);

    final_load_1 = new_load;

    fit_3rd_angle  = final_load_angle(500000:600000);
    fit_3rd_torque = final_load_1(500000:600000);

    x1 = final_load_angle(1:530000);
    y1 = fit_2nd_load_cp1(1:530000);
    x2 = final_load_angle(570000:end);
    y2 = new_load(570000:end);

    x_transition = linspace(final_load_angle(530000),final_load_angle(570000), 400001); % 过渡区间
    y_transition = spline([x1(end), x2(1)], [y1(end), y2(1)], x_transition);

    % 组合曲线
    x_combined = [x1, x_transition, x2];
    y_combined = [y1, y_transition, y2];
    
   finall_load_angle  = x_combined ;
   finall_load_torque = y_combined ;
  %%

    unloading_angle1 = 52:-0.0001:0;
    
    down_cp_1 = min(find(trial_1_unload_angle < 52)) ;
    down_cp_2 = min(find(trial_2_unload_angle < 52)) ;
    down_cp_3 = min(find(trial_3_unload_angle < 52)) ;


    k_2nd_fit_trial_1_unload = fit_gauss_3(trial_1_unload_angle(down_cp_1:end),trial_1_unload_torque(down_cp_1:end));
    k_2nd_fit_trial_2_unload = fit_gauss_3(trial_2_unload_angle(down_cp_2:end),trial_2_unload_torque(down_cp_2:end));
    k_2nd_fit_trial_3_unload = fit_gauss_3(trial_3_unload_angle(down_cp_3:end),trial_3_unload_torque(down_cp_3:end));


    fit_2nd_trial_1_unload = fit_fun_gauss(trial_1_unload_angle(down_cp_1:end),k_2nd_fit_trial_1_unload);
    fit_2nd_trial_2_unload = fit_fun_gauss(trial_2_unload_angle(down_cp_1:end),k_2nd_fit_trial_2_unload);
    fit_2nd_trial_3_unload = fit_fun_gauss(trial_3_unload_angle(down_cp_1:end),k_2nd_fit_trial_3_unload);
    
    fit_2nd_unload_cp1 = (fit_fun_gauss(unloading_angle1,k_2nd_fit_trial_1_unload) + ...
                        fit_fun_gauss(unloading_angle1,k_2nd_fit_trial_2_unload) + ...
                        fit_fun_gauss(unloading_angle1,k_2nd_fit_trial_3_unload)) ./3 ;
    
    rows   = 1:331381;
    rows_1 = 331382 : length(final_unload_torque) ;    %% 851382
    rows_2 = 1:1:520001;
    
    new_unload = zeros(size(final_unload_torque));

    new_unload(rows) = final_unload_torque(rows);
    new_unload(rows_1)   = fit_2nd_unload_cp1(rows_2);


%%
    final_load_1 = new_load;



    x1 = final_load_angle(1:530000);
    y1 = fit_2nd_load_cp1(1:530000);
    x2 = final_load_angle(570000:end);
    y2 = new_load(570000:end);

    x_transition = linspace(final_load_angle(530000),final_load_angle(570000), 400001); % 过渡区间
    y_transition = spline([x1(end), x2(1)], [y1(end), y2(1)], x_transition);

    % 组合曲线
    x_combined = [x1, x_transition, x2];
    y_combined = [y1, y_transition, y2];
    
   finall_load_angle  = x_combined ;
   finall_load_torque = y_combined ;
%%   
   % unload
    final_unload_1 = new_unload;

  
    x1 = final_unload_angle(1:321381);   %% 851382
    y1 = final_unload_1(1:321381);
    x2 = final_unload_angle(341381:end);
    y2 = final_unload_1(341381:end);

    x_transition = linspace(final_unload_angle(321381),final_unload_angle(341381), 200001); % 过渡区间
    y_transition = spline([x1(end), x2(1)], [y1(end), y2(1)], x_transition);

    % 组合曲线
    x_combined = [x1, x_transition, x2];
    y_combined = [y1, y_transition, y2];
    
   finall_unload_angle  = x_combined ;
   finall_unload_torque = y_combined ;

    %% smooth connect load unload
    
    nn = 1000;

    x1 = finall_load_angle(1:1211384-nn);   %% 1211384
    y1 = finall_load_torque(1:1211384-nn);
    x2 = finall_unload_angle(nn+1:end);
    y2 = finall_unload_torque(nn+1:end);

    x1_near = x1(1211385-nn:end);
    y1_near = x1(1211385-nn:end);
    x2_near = x2(1:nn);
    y2_near = y2(1:nn);

    x_transition = [x1_near, x2_near]; % 过渡区间
    y_transition = [y1_near, y2_near];

    x_d = linspace(x1(1211384-nn), x2(nn), 2*nn); % 过渡曲线的 x 值
    y_d = interp1(x_transition, y_transition, x_d, 'pchip'); % 使用 spline 插值

    % 拼接曲线
    final_angle = [x1, x_d, x2]; % 完整的 x 值
    final_torque  = [y1, y_d, y2];
    final_cp = find(final_torque == max(final_torque)) ;
    figure
    plot( final_angle,final_torque)
    % plot(finall_load_angle,finall_load_torque,'-','Color','#1597A5'); hold on
    % plot(finall_unload_angle,finall_unload_torque,'Linestyle','--','Color','#1597A5')
    
 %%  hysteresis
 
    x = final_angle; % 角度或位移数据
    y = final_torque; % 对应的力或扭矩数据
    
    
    x_load = x(1:final_cp); % 前半部分为加载
    [x_load_uni,ia,~] = unique(x_load);
    y_load = y(1:final_cp);
    [y_load_uni,ia,~] = unique(y_load);

    x_unload = x(final_cp +1:end); % 后半部分为卸载
    [x_unload_uni,ia,~] = unique(x_unload);
    y_unload = y(final_cp +1:end);
    [y_unload_uni,ia,~] = unique(y_unload);
    x_common = linspace(min(x), max(x), 1000000);
    y_load_interp = interp1(x_load_uni, y_load_uni, x_common, 'linear');
    y_unload_interp = interp1(x_unload_uni, y_unload_uni, x_common, 'pchip');

    hysteresis_area = abs(trapz(final_angle .*(2*pi/360),  final_torque)) ;
    hysteresis_percent = hysteresis_area /max(final_torque) * 100;
    hysteresis = vpa(hysteresis_percent,4)  
    
    %% figure  

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
    h4 = plot(final_angle(1:final_cp),final_torque(1:final_cp),'-','Color','#FFC24B');
    h4.LineWidth =2;

  
   
    %  fit 
    
    h8 = plot(final_angle(final_cp:end),final_torque(final_cp:end),'Linestyle','--','Color','#FFC24B')
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

    hysteresis_txt = text(40,2.5,'Avg hysteresis = 44.33%');
    hysteresis_txt.FontSize = 38;
%%  icc

    icc_load_angle    = linspace(0,90,10000);
    icc_load_torque_1 = interp1(trial_1_load_torque_sort, trial_1_load_angle_sort, icc_load_angle, 'linear');
    icc_load_torque_2 = interp1(trial_2_load_torque_sort, trial_2_load_angle_sort, icc_load_angle, 'linear');
    icc_load_torque_3 = interp1(trial_3_load_torque_sort, trial_3_load_angle_sort, icc_load_angle, 'linear');

    icc_unload_angle    = linspace(90,0,10000);
    icc_unload_torque_1 = interp1(trial_1_unload_torque_sort, trial_1_unload_angle_sort, icc_unload_angle, 'linear');
    icc_unload_torque_2 = interp1(trial_2_unload_torque_sort, trial_2_unload_angle_sort, icc_unload_angle, 'linear');
    icc_unload_torque_3 = interp1(trial_3_unload_torque_sort, trial_3_unload_angle_sort, icc_unload_angle, 'linear');

%%
    merge_load_angle   = max_angle_for_icc;
    merge_unload_angle = max_angle_for_icc_1;
    
    nn = 4000;
    mm = length(merge_load_angle);

    x1 = merge_load_angle(1:mm-nn);   %% 1211384
    y1 = trial_1_fit_load_torque(1:mm-nn);
    x2 = merge_load_angle(nn+1:end);
    y2 = trial_1_fit_unload_torque(nn+1:end);

    x1_near = x1(mm + 1 -nn:end);
    y1_near = x1(mm + 1 -nn:end);
    x2_near = x2(1:nn);
    y2_near = y2(1:nn);

    x_transition = [x1_near, x2_near]; % 过渡区间
    y_transition = [y1_near, y2_near];

    x_d = linspace(x1(mm-nn), x2(nn), 2*nn); % 过渡曲线的 x 值
    y_d = interp1(x_transition, y_transition, x_d, 'pchip'); % 使用 spline 插值

    % 拼接曲线
    com_x   = [x1, x_d, x2]; % 完整的 x 值
    com_y  = [y1, y_d, y2];
    
    % figure
    % plot( com_x,com_y)

    function [X,Y] = merge_R(X1,Y1,X2,Y2,N)
          
    mm = length(X1);

    xx1  = X1(1:mm-N);   %% 1211384
    yy1  = Y1(1:mm-N);
    xx2  = X2(N+1:end);
    yy2  = Y2(N+1:end);

    x1_near = xx1(mm + 1 -N:end);
    y1_near = yy1(mm + 1 -N:end);
    x2_near = xx2(1:N);
    y2_near = yy2(1:N);

    x_transition = [x1_near, x2_near]; % 过渡区间
    y_transition = [y1_near, y2_near];

    x_d = linspace(xx1(mm-N), xx2(N), 2*N); 
    y_d = interp1(x_transition, y_transition, x_d, 'pchip'); % 使用 spline 插值

    % 拼接曲线
    X   = [xx1, x_d, xx2]; % 完整的 x 值
    Y   = [yy1, y_d, yy2];          
    end

%%

  [Trial_1_x,Trial_1_y] = merge_R(merge_load_angle,trial_1_fit_load_torque,merge_unload_angle,trial_1_fit_unload_torque,32000);
  [Trial_2_x,Trial_2_y] = merge_R(merge_load_angle,trial_2_fit_load_torque,merge_unload_angle,trial_2_fit_unload_torque,5000);
  [Trial_3_x,Trial_3_y] = merge_R(merge_load_angle,trial_3_fit_load_torque,merge_unload_angle,trial_3_fit_unload_torque,57000);
  
  [Trial_1_x_uni,ia,~]  = unique(Trial_1_x);
  [Trial_1_y_uni,ia,~]  = unique(Trial_1_y);
  [Trial_2_x_uni,ia,~]  = unique(Trial_2_x);
  [Trial_2_y_uni,ia,~]  = unique(Trial_2_y);
  [Trial_3_x_uni,ia,~]  = unique(Trial_3_x);
  [Trial_3_y_uni,ia,~]  = unique(Trial_3_y);

  figure
  plot(Trial_1_x_uni,Trial_1_y_uni(2:end))
    %%
     figure
    plot(merge_load_angle,trial_1_fit_load_torque)
    hold on
    plot(merge_unload_angle,trial_1_fit_unload_torque)
%%

x_1_load    = trial_1_load_angle;
x_1_unload  = trial_1_unload_angle;
x_2_load    = trial_2_load_angle;
x_2_unload  = trial_2_unload_angle;
x_3_load    = trial_3_load_angle;
x_3_unload  = trial_3_unload_angle;
y_1_load    = trial_1_load_torque;
y_1_unload  = trial_1_unload_torque;
y_2_load    = trial_2_load_torque;
y_2_unload  = trial_2_unload_torque;
y_3_load    = trial_3_load_torque;
y_3_unload  = trial_3_unload_torque;

load_angle_start = max([min(x_1_load), min(x_2_load), min(x_3_load)]);
load_angle_end   = min([max(x_1_load), max(x_2_load), max(x_3_load)]);

unload_angle_start = max([min(x_1_unload), min(x_2_unload), min(x_3_unload)]);
unload_angle_end   = min([min(x_1_unload), min(x_2_unload), min(x_3_unload)]);

load_angle_com = linspace(load_angle_start,load_angle_end,10000);

trial_1_load_torque_com = interp1(x_1_load,y_1_load,load_angle_com,'linear');
trial_2_load_torque_com = interp1(x_2_load,y_2_load,load_angle_com,'linear');
trial_3_load_torque_com = interp1(x_3_load,y_3_load,load_angle_com,'linear');

unload_angle_com = linspace(unload_angle_start,76,10000);

trial_1_unload_torque_com = interp1(x_1_unload,y_1_unload,unload_angle_com,'linear');
trial_2_unload_torque_com = interp1(x_2_unload,y_2_unload,unload_angle_com,'linear');
trial_3_unload_torque_com = interp1(x_3_unload,y_3_unload,unload_angle_com,'linear');

trial_1_load_torque_com_sm = sgolayfilt(trial_1_load_torque_com, 3,11);
trial_2_load_torque_com_sm = sgolayfilt(trial_2_load_torque_com, 3,11);
trial_3_load_torque_com_sm = sgolayfilt(trial_3_load_torque_com, 3,11);
trial_1_unload_torque_com_sm = sgolayfilt(trial_1_unload_torque_com, 3,11);
trial_2_unload_torque_com_sm = sgolayfilt(trial_2_unload_torque_com, 3,11);
trial_3_unload_torque_com_sm = sgolayfilt(trial_3_unload_torque_com, 3,11);


%% ICC 

% load
data = [trial_1_load_torque_com_sm; trial_2_load_torque_com_sm; trial_3_load_torque_com_sm]'; % 每条曲线为列向量
[icc, ~, ~] = ICC(data, 'C-1');
load_repeatability_percent = icc * 100;


% unload
 data = [trial_1_unload_torque_com_sm; trial_2_unload_torque_com_sm; trial_3_unload_torque_com_sm]'; % 每条曲线为列向量
[icc, ~, ~] = ICC(data, 'C-1');
unload_repeatability_percent = icc * 100;

%% NRMSE
    % load
    trial_1_load_torque_com_sm_v = trial_1_load_torque_com_sm';
    trial_2_load_torque_com_sm_v = trial_2_load_torque_com_sm';
    trial_3_load_torque_com_sm_v = trial_3_load_torque_com_sm';

    nrmse12 = calculateNRMSE(trial_1_load_torque_com_sm_v, trial_2_load_torque_com_sm_v); % 曲线1和曲线2的NRMSE
    nrmse13 = calculateNRMSE(trial_1_load_torque_com_sm_v, trial_3_load_torque_com_sm_v); % 曲线1和曲线3的NRMSE
    nrmse23 = calculateNRMSE(trial_2_load_torque_com_sm_v, trial_3_load_torque_com_sm_v); % 曲线2和曲线3的NRMSE
    repeatability12 = (1 - nrmse12) * 100; % 曲线1和曲线2的可重复性
    repeatability13 = (1 - nrmse13) * 100; % 曲线1和曲线3的可重复性
    repeatability23 = (1 - nrmse23) * 100; 
    load_mean_repeatability = mean([repeatability12, repeatability13, repeatability23]);



% unload

    trial_1_unload_torque_com_sm_v = trial_1_unload_torque_com_sm';
    trial_2_unload_torque_com_sm_v = trial_2_unload_torque_com_sm';
    trial_3_unload_torque_com_sm_v = trial_3_unload_torque_com_sm';

    nrmse12 = calculateNRMSE(trial_1_unload_torque_com_sm_v, trial_2_unload_torque_com_sm_v); % 曲线1和曲线2的NRMSE
    nrmse13 = calculateNRMSE(trial_1_unload_torque_com_sm_v, trial_3_unload_torque_com_sm_v); % 曲线1和曲线3的NRMSE
    nrmse23 = calculateNRMSE(trial_2_unload_torque_com_sm_v, trial_3_unload_torque_com_sm_v); % 曲线2和曲线3的NRMSE
    repeatability12 = (1 - nrmse12) * 100; % 曲线1和曲线2的可重复性
    repeatability13 = (1 - nrmse13) * 100; % 曲线1和曲线3的可重复性
    repeatability23 = (1 - nrmse23) * 100; 
    unload_mean_repeatability = mean([repeatability12, repeatability13, repeatability23]);

function nrmse = calculateNRMSE(y_ref, y_comp)
    % 计算均方根误差（RMSE）
    rmse = sqrt(mean((y_ref - y_comp).^2));
    
    % 计算归一化均方根误差（NRMSE）
    y_range = max(y_ref) - min(y_ref); % 参考曲线的范围
    nrmse = rmse / y_range;
end

%% correlation result

icc_correlation   = [load_repeatability_percent unload_repeatability_percent ]; %%  load 3.7-82.2, unload 7-76
NRMSE_correlation = [load_mean_repeatability unload_mean_repeatability];   %%






%%

figure
plot(load_angle_com,trial_1_load_torque_com_sm,'-b')
hold on

plot(load_angle_com,trial_2_load_torque_com_sm,'--r')
plot(load_angle_com,trial_3_load_torque_com_sm,'--g')

plot(load_angle_com,trial_1_unload_torque_com_sm,'-b')


plot(load_angle_com,trial_2_unload_torque_com_sm,'--r')
plot(load_angle_com,trial_3_unload_torque_com_sm,'--g')

%%
figure
plot(trial_1_load_angle,trial_1_load_torque,'-b')
hold on

plot(trial_2_load_angle,trial_2_load_torque,'--r')
plot(trial_3_load_angle,trial_3_load_torque,'--g')
%% figure

figure();
t = tiledlayout(1,1,'TileSpacing','Compact');
plot(angle_degree_bending,T_around_B_SM);

L2 = legend('F_Y');
L2.FontSize = 38;

GG = xlabel(t,'Angle (°)'); 
GG.FontSize = 40;

GG_1 = ylabel(t,'Torque (N·m)');
GG_1.FontSize = 40;

h = title('Angle-torque profile during bending');
h.FontSize = 40;
h.FontWeight = "bold" ;



% cursor_point = [1 1500 3000 5000 6000 8000 10000 length(angle_degree_right_leg)];
% 
% % trial 1
% 
%    % loading phase 1
%         trial_1_loading_1_y   =    max(T_around_B_SM(cursor_point(2):cursor_point(3))) ;  %% max torque point
%         trial_1_loading_1_x   =    cursor_point(2) -1 + find(  T_around_B_SM(cursor_point(2):cursor_point(3))  == trial_1_loading_1_y) ;
% 
%         trial_1_loading_1_x_matrix     =  [1 1601 trial_1_loading_1_x];
%         fit_trial_1_phase_1_win_a      =  trial_1_loading_1_x_matrix(1);
%         fit_trial_1_phase_1_win_b      =  trial_1_loading_1_x_matrix(2);
%         fit_trial_1_phase_1_degree     =  angle_degree_bending(fit_trial_1_phase_1_win_a:fit_trial_1_phase_1_win_b)  ;
%         fit_trial_1_phase_1_torque     =  T_around_B_SM(fit_trial_1_phase_1_win_a:fit_trial_1_phase_1_win_b);
%         fit_x = fit_trial_1_phase_1_degree;
%         fit_y = fit_trial_1_phase_1_torque;
%         % fit
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'gauss4' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_1_phase_1  =     fitresult.a1*exp(-(( fit_x - fitresult.b1)/ fitresult.c1) .^2) + fitresult.a2*exp(-(( fit_x - fitresult.b2)/ fitresult.c2) .^2) + ...
%                                    fitresult.a3*exp(-(( fit_x - fitresult.b3)/ fitresult.c3) .^2) + fitresult.a4*exp(-(( fit_x - fitresult.b4)/ fitresult.c4) .^2) ;
% 
%         K_fit_trial_1_phase_1   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2...
%                                     fitresult.a3 fitresult.b3 fitresult.c3 fitresult.a4 fitresult.b4 fitresult.c4];
% 
%    % loading phase 2 
%         trial_1_loading_2_x_matrix     =  [1601 trial_1_loading_1_x];
%         fit_trial_1_phase_2_win_a      =  trial_1_loading_2_x_matrix(1);
%         fit_trial_1_phase_2_win_b      =  trial_1_loading_2_x_matrix(2);
%         fit_trial_1_phase_2_degree     =  angle_degree_bending(fit_trial_1_phase_2_win_a:fit_trial_1_phase_2_win_b)  ;
%         fit_trial_1_phase_2_torque     =  T_around_B_SM(fit_trial_1_phase_2_win_a:fit_trial_1_phase_2_win_b);
%         fit_x = fit_trial_1_phase_2_degree;
%         fit_y = fit_trial_1_phase_2_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype(  'power2' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_1_phase_2  =     fitresult.a*  fit_x.^fitresult.b + fitresult.c  ;
% 
%         K_fit_trial_1_phase_2   =  [fitresult.a fitresult.b fitresult.c];
% 
%         % plot(fit_trial_1_phase_2_torque );
%         % hold on;
%         % plot(fit_trial_1_phase_2);
%         % plot(fit_fun_power(fit_trial_1_phase_2_degree,K_fit_trial_1_phase_2),'-o')
% 
%    % loading phase 3 
% 
%         trial_1_loading_max_angle_y    = max(angle_degree_bending(cursor_point(2):cursor_point(3))) ;
%         trial_1_loading_max_angle_x    = cursor_point(2) -1 + find( angle_degree_bending(cursor_point(2):cursor_point(3)) == trial_1_loading_max_angle_y) ;
% 
%         trial_1_loading_3_x_matrix     =  [trial_1_loading_1_x  trial_1_loading_max_angle_x];
%         fit_trial_1_phase_3_win_a      =  trial_1_loading_3_x_matrix(1);
%         fit_trial_1_phase_3_win_b      =  trial_1_loading_3_x_matrix(2);
%         fit_trial_1_phase_3_degree     =  angle_degree_bending(fit_trial_1_phase_3_win_a:fit_trial_1_phase_3_win_b)  ;
%         fit_trial_1_phase_3_torque     =  T_around_B_SM(fit_trial_1_phase_3_win_a:fit_trial_1_phase_3_win_b);
%         fit_x = fit_trial_1_phase_3_degree;
%         fit_y = fit_trial_1_phase_3_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'logistic' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 86];
%         opts.StartPoint = [40.4295581921676 -2.26381247712446 88];
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_1_phase_3  =     fitresult.a ./(1 + exp(-fitresult.b.*(fit_x-fitresult.c)))  ;
% 
%         K_fit_trial_1_phase_3   =  [fitresult.a fitresult.b fitresult.c];
% 
%         % figure
%         % plot(fit_trial_1_phase_3_torque );
%         % hold on;
%         % plot(fit_trial_1_phase_3,'-o');
%         % plot(fit_fun_log(fit_trial_1_phase_3_degree,K_fit_trial_1_phase_3),'-o')
% 
% 
% 
%            % unloading phase 1 
% 
%         trial_1_unloading_x =  2260 ;
%         trial_1_unloading_1_x_matrix     =  [trial_1_loading_max_angle_x  2260];
%         fit_trial_1_phase_4_win_a      =  trial_1_unloading_1_x_matrix(1);
%         fit_trial_1_phase_4_win_b      =  trial_1_unloading_1_x_matrix(2);
%         fit_trial_1_phase_4_degree     =  angle_degree_bending(fit_trial_1_phase_4_win_a:fit_trial_1_phase_4_win_b)  ;
%         fit_trial_1_phase_4_torque     =  T_around_B_SM(fit_trial_1_phase_4_win_a:fit_trial_1_phase_4_win_b);
%         fit_x = fit_trial_1_phase_4_degree;
%         fit_y = fit_trial_1_phase_4_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'gauss3' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [38.2151730262292 84.9724901678257 0.57783091782667 31.2650164496321 83.7851090913557 0.602708334453609 26.2878559685646 82.6278233632257 0.561838561245442];
% 
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_1_phase_4 =     fitresult.a1*exp(-((fit_x- fitresult.b1)/ fitresult.c1) .^2) + fitresult.a2*exp(-((fit_x- fitresult.b2)/ fitresult.c2) .^2) + ...
%                                   fitresult.a3*exp(-((fit_x- fitresult.b3)/ fitresult.c3) .^2) ;
%         K_fit_trial_1_phase_4   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
%         %         figure
%         % plot(fit_trial_1_phase_4_torque );
%         % hold on;
%         % plot(fit_trial_1_phase_4,'-o');
%         % plot(fit_fun_gauss(fit_trial_1_phase_4_degree,K_fit_trial_1_phase_4),'-o')
% 
%    % unloading phase 2
% 
%         trial_1_unloading_x =  2260 ;
%         trial_1_unloading_min_torque_y = min(T_around_B_SM(cursor_point(3):cursor_point(4))) ;
%         trial_1_unloading_min_torque_x = cursor_point(3) -1 + find( T_around_B_SM(cursor_point(3):cursor_point(4)) == trial_1_unloading_min_torque_y) ; 
% 
%         trial_1_unloading_2_x_matrix     =  [2260 trial_1_unloading_min_torque_x ];
%         fit_trial_1_phase_5_win_a      =  trial_1_unloading_2_x_matrix(1);
%         fit_trial_1_phase_5_win_b      =  trial_1_unloading_2_x_matrix(2);
%         fit_trial_1_phase_5_degree     =  angle_degree_bending(fit_trial_1_phase_5_win_a:fit_trial_1_phase_5_win_b)  ;
%         fit_trial_1_phase_5_torque     =  T_around_B_SM(fit_trial_1_phase_5_win_a:fit_trial_1_phase_5_win_b);
%         fit_x = fit_trial_1_phase_5_degree;
%         fit_y = fit_trial_1_phase_5_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
% 
%         % Set up fittype and options.
%         ft = fittype( 'gauss3' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [21.0213253316517 79.6954446439272 5.80187232818257 16.9543452126289 67.4901827542207 5.46669105373693 14.1050982225823 56.905938920927 5.02757516305261];
%         [fitresult, gof] = fit( xData, yData, ft, opts );
%         fit_trial_1_phase_5 =     fitresult.a1 *exp(-((fit_x - fitresult.b1) ./ fitresult.c1) .^2) + fitresult.a2 *exp(-((fit_x - fitresult.b2) ./ fitresult.c2) .^2) + ...
%                                   fitresult.a3 *exp(-((fit_x - fitresult.b3) ./ fitresult.c3) .^2) ;
%         K_fit_trial_1_phase_5   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
% 
% 
%         % figure
%         % plot(fit_trial_1_phase_5_degree ,fit_trial_1_phase_5_torque );
%         % hold on;
%         % plot(fit_trial_1_phase_5_degree ,fit_trial_1_phase_5,'-o');
%         % plot(fit_trial_1_phase_5_degree ,fit_fun_gauss(fit_trial_1_phase_5_degree,K_fit_trial_1_phase_5),'-o');
% %     trial 1 whole fit
%         deg = 0:0.01:85 ;
%         v = zeros(size(deg)) ;
%         for i = 1:length(deg)
%             if deg(i) <= 43.9277
%                 v(i) = fit_fun_gauss(deg(i),K_fit_trial_1_phase_1) ;
%             elseif   (deg(i) >= 43.9277)&(deg(i) <= 83.8690)
%                 v(i) = fit_fun_power(deg(i),K_fit_trial_1_phase_2) ;
%             else
%                 v(i) = fit_fun_log(deg(i),K_fit_trial_1_phase_3) ;
%             end
%         end
% 
%         cp1 = [85 fit_fun_log(85,K_fit_trial_1_phase_3)];
%         figure
% 
%         % plot(angle_degree_bending(1: trial_1_unloading_min_torque_x), T_around_B_SM(1: trial_1_unloading_min_torque_x),'k');
%         hold on
%         plot(deg,v,'r');
% 
%         % plot(deg,smoothdata(v,'rlowess',200),'--r')     
% 
%         deg_1 = 85 : -0.01 :0 ;
%         u = zeros(size(deg_1)) ;
%         for i = 1:length(deg_1)
%             if deg_1(i) >= 79.6954 
%                 u(i) = fit_fun_gauss(deg_1(i),K_fit_trial_1_phase_4) ;
%             elseif   (deg_1(i) < 79.6954)
%                 u(i) = fit_fun_gauss(deg_1(i),K_fit_trial_1_phase_5) ;
% 
%             end
%         end
% 
% 
%          plot(deg_1,u,'-b')
%        % plot(deg_1,smoothdata(u,'movmedian',100),'-b')
% 
%        merge_curve_x = [84  84.5 85 ];
%        merge_curve_y = [84  84.5 85 ];
%  %%  trial 2
% 
%         cursor_point = [1 1500 3000 5000 6000 8000 10000 length(angle_degree_right_leg)];
%    % loading phase 1
%         trial_2_loading_1_y   =    max(T_around_B_SM(cursor_point(4):cursor_point(5))) ;  %% max torque point
%         trial_2_loading_1_x   =    cursor_point(4) -1 + find(  T_around_B_SM(cursor_point(4):cursor_point(5))  == trial_2_loading_1_y) ;
% 
%         trial_2_loading_1_x_matrix     =  [trial_1_unloading_min_torque_x, 5000,trial_2_loading_1_x];
%         fit_trial_2_phase_1_win_a      =  trial_2_loading_1_x_matrix(1);
%         fit_trial_2_phase_1_win_b      =  trial_2_loading_1_x_matrix(2);
%         fit_trial_2_phase_1_degree     =  angle_degree_bending(fit_trial_2_phase_1_win_a:fit_trial_2_phase_1_win_b)  ;
%         fit_trial_2_phase_1_torque     =  T_around_B_SM(fit_trial_2_phase_1_win_a:fit_trial_2_phase_1_win_b);
%         fit_x = fit_trial_2_phase_1_degree;
%         fit_y = fit_trial_2_phase_1_torque;
%         % fit
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'gauss4' );
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [27.3786883450675 47.992182342696 2.57518479760102 23.161652921562 42.7611911731822 2.47633949575929 20.7953680421053 37.8801778557129 2.19549672595256 17.6595915857792 33.6592667080689 2.21644551168485];
% 
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_2_phase_1  =     fitresult.a1*exp(-(( fit_x - fitresult.b1)/ fitresult.c1) .^2) + fitresult.a2*exp(-(( fit_x - fitresult.b2)/ fitresult.c2) .^2) + ...
%                                    fitresult.a3*exp(-(( fit_x - fitresult.b3)/ fitresult.c3) .^2) + fitresult.a4*exp(-(( fit_x - fitresult.b4)/ fitresult.c4) .^2) ;
% 
%         K_fit_trial_2_phase_1   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2...
%                                     fitresult.a3 fitresult.b3 fitresult.c3 fitresult.a4 fitresult.b4 fitresult.c4];
% %
%    % loading phase 2 
%         trial_2_loading_2_x_matrix     =  [5001 trial_2_loading_1_x];
%         fit_trial_2_phase_2_win_a      =  trial_2_loading_2_x_matrix(1);
%         fit_trial_2_phase_2_win_b      =  trial_2_loading_2_x_matrix(2);
%         fit_trial_2_phase_2_degree     =  angle_degree_bending(fit_trial_2_phase_2_win_a:fit_trial_2_phase_2_win_b)  ;
%         fit_trial_2_phase_2_torque     =  T_around_B_SM(fit_trial_2_phase_2_win_a:fit_trial_2_phase_2_win_b);
%         fit_x = fit_trial_2_phase_2_degree;
%         fit_y = fit_trial_2_phase_2_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype(  'power2' );
% 
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.StartPoint = [0.894380041214355 0.877420596228625 -0.214739975730606];
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_2_phase_2  =     fitresult.a*  fit_x.^fitresult.b + fitresult.c  ;
% 
%         K_fit_trial_2_phase_2   =  [fitresult.a fitresult.b fitresult.c];
% 
%         % plot(fit_trial_2_phase_2_torque );
%         % hold on;
%         % plot(fit_trial_2_phase_2);
%         % plot(fit_fun_power(fit_trial_2_phase_2_degree,K_fit_trial_2_phase_2),'-o')
% %
%    % loading phase 3 
% 
%         trial_2_loading_max_angle_y    = max(angle_degree_bending(cursor_point(4):cursor_point(5))) ;
%         trial_2_loading_max_angle_x    = cursor_point(4) -1 + find( angle_degree_bending(cursor_point(4):cursor_point(5)) == trial_2_loading_max_angle_y) ;
% 
%         trial_2_loading_3_x_matrix     =  [trial_2_loading_1_x  trial_2_loading_max_angle_x];
%         fit_trial_2_phase_3_win_a      =  trial_2_loading_3_x_matrix(1);
%         fit_trial_2_phase_3_win_b      =  trial_2_loading_3_x_matrix(2);
%         fit_trial_2_phase_3_degree     =  angle_degree_bending(fit_trial_2_phase_3_win_a:fit_trial_2_phase_3_win_b)  ;
%         fit_trial_2_phase_3_torque     =  T_around_B_SM(fit_trial_2_phase_3_win_a:fit_trial_2_phase_3_win_b);
%         fit_x = fit_trial_2_phase_3_degree;
%         fit_y = fit_trial_2_phase_3_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'logistic' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.StartPoint = [41.719614825908 -3.0024164831206 92];
%         opts.Display = 'Off';
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_2_phase_3  =     fitresult.a ./(1 + exp(-fitresult.b.*(fit_x-fitresult.c)))  ;
% 
%         K_fit_trial_2_phase_3   =  [fitresult.a fitresult.b fitresult.c];
% 
%         % figure
%         % plot(fit_trial_2_phase_3_torque );
%         % hold on;
%         % plot(fit_trial_2_phase_3,'-o');
%         % plot(fit_fun_log(fit_trial_2_phase_3_degree,K_fit_trial_2_phase_3),'-o')
% 
% 
% %
%            % unloading phase 1 
% 
%         trial_2_unloading_x =  5583 ;
%         trial_2_unloading_1_x_matrix     =  [trial_2_loading_max_angle_x  trial_2_unloading_x];
%         fit_trial_2_phase_4_win_a      =  trial_2_unloading_1_x_matrix(1);
%         fit_trial_2_phase_4_win_b      =  trial_2_unloading_1_x_matrix(2);
%         fit_trial_2_phase_4_degree     =  angle_degree_bending(fit_trial_2_phase_4_win_a:fit_trial_2_phase_4_win_b)  ;
%         fit_trial_2_phase_4_torque     =  T_around_B_SM(fit_trial_2_phase_4_win_a:fit_trial_2_phase_4_win_b);
%         fit_x = fit_trial_2_phase_4_degree;
%         fit_y = fit_trial_2_phase_4_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'gauss3' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [38.2151730262292 84.9724901678257 0.57783091782667 31.2650164496321 83.7851090913557 0.602708334453609 26.2878559685646 82.6278233632257 0.561838561245442];
% 
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_2_phase_4 =     fitresult.a1*exp(-((fit_x- fitresult.b1)/ fitresult.c1) .^2) + fitresult.a2*exp(-((fit_x- fitresult.b2)/ fitresult.c2) .^2) + ...
%                                   fitresult.a3*exp(-((fit_x- fitresult.b3)/ fitresult.c3) .^2) ;
%         K_fit_trial_2_phase_4   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
%         %         figure
%         % plot(fit_trial_2_phase_4_torque );
%         % hold on;
%         % plot(fit_trial_2_phase_4,'-o');
%         % plot(fit_fun_gauss(fit_trial_2_phase_4_degree,K_fit_trial_2_phase_4),'-o')
% 
%         %
%    % unloading phase 2
% 
%         trial_2_unloading_x =   5583 ;
%         trial_2_unloading_min_torque_y = min(T_around_B_SM(cursor_point(5):cursor_point(6))) ;
%         trial_2_unloading_min_torque_x = cursor_point(5) -1 + find( T_around_B_SM(cursor_point(5):cursor_point(6)) == trial_2_unloading_min_torque_y) ; 
% 
%         trial_2_unloading_2_x_matrix     =  [5583,trial_2_unloading_min_torque_x];
%         fit_trial_2_phase_5_win_a      =  trial_2_unloading_2_x_matrix(1);
%         fit_trial_2_phase_5_win_b      =  trial_2_unloading_2_x_matrix(2);
%         fit_trial_2_phase_5_degree     =  angle_degree_bending(fit_trial_2_phase_5_win_a:fit_trial_2_phase_5_win_b)  ;
%         fit_trial_2_phase_5_torque     =  T_around_B_SM(fit_trial_2_phase_5_win_a:fit_trial_2_phase_5_win_b);
%         fit_x = fit_trial_2_phase_5_degree;
%         fit_y = fit_trial_2_phase_5_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
% 
%         % Set up fittype and options.
%         ft = fittype( 'gauss3' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [21.0213253316517 79.6954446439272 5.80187232818257 16.9543452126289 67.4901827542207 5.46669105373693 14.1050982225823 56.905938920927 5.02757516305261];
%         [fitresult, gof] = fit( xData, yData, ft, opts );
%         fit_trial_2_phase_5 =     fitresult.a1 *exp(-((fit_x - fitresult.b1) ./ fitresult.c1) .^2) + fitresult.a2 *exp(-((fit_x - fitresult.b2) ./ fitresult.c2) .^2) + ...
%                                   fitresult.a3 *exp(-((fit_x - fitresult.b3) ./ fitresult.c3) .^2) ;
%         K_fit_trial_2_phase_5   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
% 
% 
%         % figure
%         % plot(fit_trial_2_phase_5_degree ,fit_trial_2_phase_5_torque );
%         % hold on;
%         % plot(fit_trial_2_phase_5_degree ,fit_trial_2_phase_5,'-o');
%         % plot(fit_trial_2_phase_5_degree ,fit_fun_gauss(fit_trial_2_phase_5_degree,K_fit_trial_2_phase_5),'-o');
% 
% 
% 
% %    trial 2 whole fit
% 
%         v = zeros(size(deg)) ;
%         pp1 = angle_degree_bending(5000);
%         pp2 = angle_degree_bending(trial_2_loading_1_x);
%         pp3 = angle_degree_bending(trial_2_loading_max_angle_x);
%         deg = 0:0.01:pp3 ;
%         for i = 1:length(deg)
%             if deg(i) <= pp1
%                 v(i) = fit_fun_gauss(deg(i),K_fit_trial_2_phase_1) ;
%             elseif   (deg(i) >= pp1)&(deg(i) <= pp2)
%                 v(i) = fit_fun_power(deg(i),K_fit_trial_2_phase_2) ;
%             else
%                 v(i) = fit_fun_log(deg(i),K_fit_trial_2_phase_3) ;
%             end
%         end
% 
%         % cp1 = [85 fit_fun_log(85,K_fit_trial_2_phase_3)];
%         figure
% 
%         % plot(angle_degree_bending(1: trial_1_unloading_min_torque_x), T_around_B_SM(1: trial_1_unloading_min_torque_x),'k');
%         hold on
%         plot(deg,v,'r');
% 
%         % plot(deg,smoothdata(v,'rlowess',200),'--r')     
% 
%         pp4 = angle_degree_bending(trial_2_unloading_x);
%         deg_1 = pp3 : -0.01 :0 ;
%         u = zeros(size(deg_1)) ;
%         for i = 1:length(deg_1)
%             if deg_1(i) >= pp4 
%                 u(i) = fit_fun_gauss(deg_1(i),K_fit_trial_2_phase_4) ;
%             elseif   (deg_1(i) < pp4)
%                 u(i) = fit_fun_gauss(deg_1(i),K_fit_trial_2_phase_5) ;
% 
%             end
%         end
% 
% 
%          plot(deg_1,u,'-b')
%      %  v1 = v;
%      % v1(4250:4550) = smoothdata(v1(4250:4550), 'gaussian',50);
%      % v1(8200:8475) = smoothdata(v1(8200:8475), 'sgolay',175);
%      % figure
%      % plot(deg,v1)
%      % hold on
%      % % plot(deg,v,'r')
%      % plot(deg_1,u)
% %%  trial 3
% 
%         cursor_point = [1 1500 3000 5000 6000 8000 10000 length(angle_degree_right_leg)];
%    % loading phase 1
%         trial_3_loading_1_y   =    max(T_around_B_SM(cursor_point(6):cursor_point(7))) ;  %% max torque point
%         trial_3_loading_1_x   =    cursor_point(6) -1 + find(  T_around_B_SM(cursor_point(6):cursor_point(7))  == trial_3_loading_1_y) ;
% 
%         trial_3_loading_1_x_matrix     =  [trial_2_unloading_min_torque_x, 7928,trial_3_loading_1_x];
%         fit_trial_3_phase_1_win_a      =  trial_3_loading_1_x_matrix(1);
%         fit_trial_3_phase_1_win_b      =  trial_3_loading_1_x_matrix(2);
%         fit_trial_3_phase_1_degree     =  angle_degree_bending(fit_trial_3_phase_1_win_a:fit_trial_3_phase_1_win_b)  ;
%         fit_trial_3_phase_1_torque     =  T_around_B_SM(fit_trial_3_phase_1_win_a:fit_trial_3_phase_1_win_b);
%         fit_x = fit_trial_3_phase_1_degree;
%         fit_y = fit_trial_3_phase_1_torque;
%         % fit
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'gauss4' );
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [25.1815457484157 45.4717521635059 2.37113370731611 21.6895545606252 40.4978490619326 2.18095443116188 19.1184670534952 36.1498112859283 1.95703639629606 16.0284983055687 32.2813337907608 2.0222776674883];
% 
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_3_phase_1  =     fitresult.a1*exp(-(( fit_x - fitresult.b1)/ fitresult.c1) .^2) + fitresult.a2*exp(-(( fit_x - fitresult.b2)/ fitresult.c2) .^2) + ...
%                                    fitresult.a3*exp(-(( fit_x - fitresult.b3)/ fitresult.c3) .^2) + fitresult.a4*exp(-(( fit_x - fitresult.b4)/ fitresult.c4) .^2) ;
% 
%         K_fit_trial_3_phase_1   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2...
%                                     fitresult.a3 fitresult.b3 fitresult.c3 fitresult.a4 fitresult.b4 fitresult.c4];
% %
%    % loading phase 2 
%         trial_3_loading_2_x_matrix     =  [7928 trial_3_loading_1_x];
%         fit_trial_3_phase_2_win_a      =  trial_3_loading_2_x_matrix(1);
%         fit_trial_3_phase_2_win_b      =  trial_3_loading_2_x_matrix(2);
%         fit_trial_3_phase_2_degree     =  angle_degree_bending(fit_trial_3_phase_2_win_a:fit_trial_3_phase_2_win_b)  ;
%         fit_trial_3_phase_2_torque     =  T_around_B_SM(fit_trial_3_phase_2_win_a:fit_trial_3_phase_2_win_b);
%         fit_x = fit_trial_3_phase_2_degree;
%         fit_y = fit_trial_3_phase_2_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype(  'power2' );
% 
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.StartPoint = [0.515126727797758 1.01188693247257 -0.297107382952972];
% 
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_3_phase_2  =     fitresult.a*  fit_x.^fitresult.b + fitresult.c  ;
% 
%         K_fit_trial_3_phase_2   =  [fitresult.a fitresult.b fitresult.c];
% 
%         % plot(fit_trial_3_phase_2_torque );
%         % hold on;
%         % plot(fit_trial_3_phase_2);
%         % plot(fit_fun_power(fit_trial_3_phase_2_degree,K_fit_trial_3_phase_2),'-o')
% %
%    % loading phase 3 
% 
%         trial_3_loading_max_angle_y    = max(angle_degree_bending(cursor_point(6):cursor_point(7))) ;
%         trial_3_loading_max_angle_x    = cursor_point(6) -1 + find( angle_degree_bending(cursor_point(6):cursor_point(7)) == trial_3_loading_max_angle_y) ;
% 
%         trial_3_loading_3_x_matrix     =  [trial_3_loading_1_x  trial_3_loading_max_angle_x];
%         fit_trial_3_phase_3_win_a      =  trial_3_loading_3_x_matrix(1);
%         fit_trial_3_phase_3_win_b      =  trial_3_loading_3_x_matrix(2);
%         fit_trial_3_phase_3_degree     =  angle_degree_bending(fit_trial_3_phase_3_win_a:fit_trial_3_phase_3_win_b)  ;
%         fit_trial_3_phase_3_torque     =  T_around_B_SM(fit_trial_3_phase_3_win_a:fit_trial_3_phase_3_win_b);
%         fit_x = fit_trial_3_phase_3_degree;
%         fit_y = fit_trial_3_phase_3_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'logistic4' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
% 
%         opts.StartPoint = [39.1549323171069 -1297.31065865901 82.4171072673322 40.6086518550094];
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_3_phase_3  =     fitresult.d + (fitresult.a-fitresult.d)/(1 + (x./fitresult.c).^fitresult.b) ;
% 
%         K_fit_trial_3_phase_3   =  [fitresult.a fitresult.b fitresult.c fitresult.d];
% 
%         % figure
%         % plot(fit_trial_3_phase_3_torque );
%         % hold on;
%         % plot(fit_trial_3_phase_3,'-o');
%         % plot(fit_fun_log_4(fit_trial_3_phase_3_degree,K_fit_trial_3_phase_3),'-o')
% %%
% fit_fun_log_4(84.6,K_fit_trial_3_phase_3)
% %%
% 
% %
%            % unloading phase 1 
% 
%         trial_3_unloading_x =  8574 ;
%         trial_3_unloading_1_x_matrix     =  [trial_3_loading_max_angle_x  trial_3_unloading_x];
%         fit_trial_3_phase_4_win_a      =  trial_3_unloading_1_x_matrix(1);
%         fit_trial_3_phase_4_win_b      =  trial_3_unloading_1_x_matrix(2);
%         fit_trial_3_phase_4_degree     =  angle_degree_bending(fit_trial_3_phase_4_win_a:fit_trial_3_phase_4_win_b)  ;
%         fit_trial_3_phase_4_torque     =  T_around_B_SM(fit_trial_3_phase_4_win_a:fit_trial_3_phase_4_win_b);
%         fit_x = fit_trial_3_phase_4_degree;
%         fit_y = fit_trial_3_phase_4_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
%         ft = fittype( 'gauss3' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [39.1549323171069 82.4730854105189 0.716065290574429 30.3742557855794 81.0359225719278 0.720031798229075 25.8146534856674 79.5382543149416 0.705017511644378];
% 
%         [fitresult, gof] = fit( xData, yData, ft, opts );
% 
%         fit_trial_3_phase_4 =     fitresult.a1*exp(-((fit_x- fitresult.b1)/ fitresult.c1) .^2) + fitresult.a2*exp(-((fit_x- fitresult.b2)/ fitresult.c2) .^2) + ...
%                                   fitresult.a3*exp(-((fit_x- fitresult.b3)/ fitresult.c3) .^2) ;
%         K_fit_trial_3_phase_4   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
%         %         figure
%         % plot(fit_trial_3_phase_4_torque );
%         % hold on;
%         % plot(fit_trial_3_phase_4,'-o');
%         % plot(fit_fun_gauss(fit_trial_3_phase_4_degree,K_fit_trial_3_phase_4),'-o')
% 
%         %
%    % unloading phase 2
% 
%         trial_3_unloading_x =   8574 ;
%         trial_3_unloading_min_torque_y = min(T_around_B_SM(cursor_point(7):cursor_point(8))) ;
%         trial_3_unloading_min_torque_x = cursor_point(7) -1 + find( T_around_B_SM(cursor_point(7):cursor_point(8)) == trial_3_unloading_min_torque_y) ; 
% 
%         trial_3_unloading_2_x_matrix     =  [8574,trial_3_unloading_min_torque_x];
%         fit_trial_3_phase_5_win_a      =  trial_3_unloading_2_x_matrix(1);
%         fit_trial_3_phase_5_win_b      =  trial_3_unloading_2_x_matrix(2);
%         fit_trial_3_phase_5_degree     =  angle_degree_bending(fit_trial_3_phase_5_win_a:fit_trial_3_phase_5_win_b)  ;
%         fit_trial_3_phase_5_torque     =  T_around_B_SM(fit_trial_3_phase_5_win_a:fit_trial_3_phase_5_win_b);
%         fit_x = fit_trial_3_phase_5_degree;
%         fit_y = fit_trial_3_phase_5_torque;
%         [xData, yData] = prepareCurveData( fit_x, fit_y );
% 
%         % Set up fittype and options.
%         ft = fittype( 'gauss3' );
%         opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
%         opts.Display = 'Off';
%         opts.Lower = [-Inf -Inf 0 -Inf -Inf 0 -Inf -Inf 0];
%         opts.StartPoint = [19.4833308525243 76.196743715157 5.51969211131866 16.1496947500731 65.1744439364629 5.1799873937057 13.5460958718273 55.1486611606915 4.84532131302877];
%         [fitresult, gof] = fit( xData, yData, ft, opts );
%         fit_trial_3_phase_5 =     fitresult.a1 *exp(-((fit_x - fitresult.b1) ./ fitresult.c1) .^2) + fitresult.a2 *exp(-((fit_x - fitresult.b2) ./ fitresult.c2) .^2) + ...
%                                   fitresult.a3 *exp(-((fit_x - fitresult.b3) ./ fitresult.c3) .^2) ;
%         K_fit_trial_3_phase_5   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
% 
% 
%         % figure
%         % plot(fit_trial_3_phase_5_degree ,fit_trial_3_phase_5_torque );
%         % hold on;
%         % plot(fit_trial_3_phase_5_degree ,fit_trial_3_phase_5,'-o');
%         % plot(fit_trial_3_phase_5_degree ,fit_fun_gauss(fit_trial_3_phase_5_degree,K_fit_trial_3_phase_5),'-o');
%  %
%  % trial 3 whole fit
% 
%         v = zeros(size(deg)) ;
%         pp1 = angle_degree_bending(7928);
%         pp2 = angle_degree_bending(trial_3_loading_1_x);
%         pp3 = angle_degree_bending(trial_3_loading_max_angle_x);
%         deg = 0:0.01:pp3 ;
%         for i = 1:length(deg)
%             if deg(i) <= pp1
%                 v(i) = fit_fun_gauss(deg(i),K_fit_trial_3_phase_1) ;
%             elseif   (deg(i) >= pp1)&(deg(i) <= pp2)
%                 v(i) = fit_fun_power(deg(i),K_fit_trial_3_phase_2) ;
%             else
%                 v(i) = fit_fun_log_4(deg(i),K_fit_trial_3_phase_3) ;
%             end
%         end
% 
%         % cp1 = [85 fit_fun_log(85,K_fit_trial_3_phase_3)];
%         figure
% 
%         % plot(angle_degree_bending(1: trial_1_unloading_min_torque_x), T_around_B_SM(1: trial_1_unloading_min_torque_x),'k');
%         hold on
%         plot(deg,v,'r');
% 
%         % plot(deg,smoothdata(v,'rlowess',200),'--r')     
% 
%         pp4 = angle_degree_bending(trial_3_unloading_x);
%         deg_1 = pp3 : -0.01 :0 ;
%         u = zeros(size(deg_1)) ;
%         for i = 1:length(deg_1)
%             if deg_1(i) >= pp4 
%                 u(i) = fit_fun_gauss(deg_1(i),K_fit_trial_3_phase_4) ;
%             elseif   (deg_1(i) < pp4)
%                 u(i) = fit_fun_gauss(deg_1(i),K_fit_trial_3_phase_5) ;
% 
%             end
%         end
% 
% 
%          plot(deg_1,u,'-b')
% 
% %%   average fit
%      time_matrix = [1601, trial_1_loading_1_x, trial_1_loading_max_angle_x, 2260;
%                     5001, trial_2_loading_1_x, trial_2_loading_max_angle_x, 5583;
%                     7928, trial_3_loading_1_x, trial_3_loading_max_angle_x, 8574];
% 
%      deg_matrix = zeros(3,4);
%      for i = 1:3
%          for j =1:4
%              deg_matrix(i,j) = angle_degree_bending(time_matrix(i,j)) ;
%          end 
%      end
% 
%      avg_deg   = sum(deg_matrix, 1)./3;
% 
%      deg_load = 0:0.01:avg_deg(3) ;
%      avg_fit_load = zeros(size(1:length(deg_load)));
%      for i = 1:length(deg_load)
%             if deg_load(i) <= avg_deg(1)
%                 avg_fit_load(i) = (fit_fun_gauss(deg_load(i),K_fit_trial_1_phase_1) + fit_fun_gauss(deg_load(i),K_fit_trial_2_phase_1) + fit_fun_gauss(deg_load(i),K_fit_trial_3_phase_1) ) ./3;
%             elseif   (deg_load(i) >= avg_deg(1))&(deg_load(i) <= avg_deg(2))
%                 avg_fit_load(i) = (fit_fun_power(deg_load(i),K_fit_trial_1_phase_2) + fit_fun_power(deg_load(i),K_fit_trial_2_phase_2) + fit_fun_power(deg_load(i),K_fit_trial_3_phase_2)) ./3 ;
%             else
%                 avg_fit_load(i) = (fit_fun_log(deg_load(i),K_fit_trial_1_phase_3) + fit_fun_log(deg_load(i),K_fit_trial_2_phase_3) + fit_fun_log_4(deg_load(i),K_fit_trial_3_phase_3)) ./3 ;
%             end
%      end
% 
%      figure()
%      plot(deg_load, avg_fit_load)
%      hold on
% 
%      deg_unload = avg_deg(3) : -0.01 :0 ;
%      avg_fit_unload = zeros(size(1:length(deg_unload)));
% 
%         for i = 1:length(deg_unload)
%             if deg_unload(i) >= avg_deg(4) 
%                 avg_fit_unload(i) = (fit_fun_gauss(deg_unload(i),K_fit_trial_1_phase_4) + fit_fun_gauss(deg_unload(i),K_fit_trial_2_phase_4) + fit_fun_gauss(deg_unload(i),K_fit_trial_3_phase_4)) ./3;
%             elseif   (deg_unload(i) < avg_deg(4))
%                 avg_fit_unload(i) = (fit_fun_gauss(deg_unload(i),K_fit_trial_1_phase_5) + fit_fun_gauss(deg_unload(i),K_fit_trial_2_phase_5) + fit_fun_gauss(deg_unload(i),K_fit_trial_3_phase_5))./3;
% 
%             end
%         end
% 
%         plot(deg_unload, avg_fit_unload)
% plot(angle_degree_bending, T_around_B_SM,'o')
% %%
% 
%      deg_load = 0:0.01:avg_deg(3) ;
%      avg_fit_load = zeros(size(1:length(deg_load)));
%      for i = 1:length(deg_load)
%             if deg_load(i) <= avg_deg(1)
%                 avg_fit_load(i) = (fit_fun_gauss(deg_load(i),K_fit_trial_1_phase_1) + fit_fun_gauss(deg_load(i),K_fit_trial_2_phase_1) + fit_fun_gauss(deg_load(i),K_fit_trial_3_phase_1) ) ./3;
%             elseif   (deg_load(i) >= avg_deg(1))&(deg_load(i) <= avg_deg(2))
%                 avg_fit_load(i) = (fit_fun_power(deg_load(i),K_fit_trial_1_phase_2) + fit_fun_power(deg_load(i),K_fit_trial_2_phase_2) + fit_fun_power(deg_load(i),K_fit_trial_3_phase_2)) ./3 ;
%             else
%                 avg_fit_load(i) =  fit_fun_log(deg_load(i),K_fit_trial_1_phase_3)  ;
%             end
%      end
% 
%      figure()
%      plot(deg_load, avg_fit_load)
%      hold on
% 
%      deg_unload = avg_deg(3) : -0.01 :0 ;
%      avg_fit_unload = zeros(size(1:length(deg_unload)));
% 
%         for i = 1:length(deg_unload)
%             if deg_unload(i) >= avg_deg(4) 
%                 avg_fit_unload(i) = (fit_fun_gauss(deg_unload(i),K_fit_trial_1_phase_4) + fit_fun_gauss(deg_unload(i),K_fit_trial_2_phase_4) + fit_fun_gauss(deg_unload(i),K_fit_trial_3_phase_4)) ./3;
%             elseif   (deg_unload(i) < avg_deg(4))
%                 avg_fit_unload(i) = (fit_fun_gauss(deg_unload(i),K_fit_trial_1_phase_5) + fit_fun_gauss(deg_unload(i),K_fit_trial_2_phase_5) + fit_fun_gauss(deg_unload(i),K_fit_trial_3_phase_5))./3;
% 
%             end
%         end
% 
%         plot(deg_unload, avg_fit_unload)
%  plot(angle_degree_bending, T_around_B_SM,'r')




%%
% zero_matrix = zeros(size(Real_chest_hori_front_top));
% 
% F_F1  = [Real_chest_hori_front_top zero_matrix];
% F_F2  = [Real_chest_hori_front_bot zero_matrix];
% F_F3  = [Real_chest_hori_back      zero_matrix];
% F_F4  = [zero_matrix  -Real_chest_verti_front];
% F_F5  = [zero_matrix  -Real_chest_verti_back];
% F_F6  = [Real_hip_hori_top .*cos(angle_rad_lower_back_joint) ,   -Real_hip_hori_top.*sin(angle_rad_lower_back_joint)];
% F_F7  = [Real_hip_hori_bot .*cos(angle_rad_lower_back_joint) ,   -Real_hip_hori_bot.*sin(angle_rad_lower_back_joint)];
% F_F8  = [-Real_hip_verti_front .*sin(angle_rad_lower_back_joint) ,   -Real_hip_verti_front.*cos(angle_rad_lower_back_joint)];
% F_F9  = [-Real_hip_verti_back .*sin(angle_rad_lower_back_joint) ,   -Real_hip_verti_back.*cos(angle_rad_lower_back_joint)];
% F_F10 = [Real_leg_hori_top .*cos(angle_rad_bending) ,   -Real_leg_hori_top.*sin(angle_rad_bending)];
% F_F11 = [Real_leg_hori_bot .*cos(angle_rad_bending) ,   -Real_leg_hori_bot.*sin(angle_rad_bending)];
% F_F12 = [-Real_leg_verti_back .*sin(angle_rad_bending) ,   -Real_leg_verti_back.*cos(angle_rad_bending)];
% F_F13 = [-Real_leg_verti_front .*sin(angle_rad_bending) ,   -Real_leg_verti_front.*cos(angle_rad_bending)];
% 
% 
% FX = F_F1(:,1) + F_F2(:,1) + F_F3(:,1)  + F_F4(:,1)  + F_F5(:,1)  + F_F6(:,1) + F_F7(:,1) + ...
%      F_F8(:,1) + F_F9(:,1) + F_F10(:,1) + F_F11(:,1) + F_F12(:,1) + F_F13(:,1) ;
% FY = F_F1(:,2) + F_F2(:,2) + F_F3(:,2)  + F_F4(:,2)  + F_F5(:,2)  + F_F6(:,2) + F_F7(:,2) + ...
%      F_F8(:,2) + F_F9(:,2) + F_F10(:,2) + F_F11(:,2) + F_F12(:,2) + F_F13(:,2) ;
% 
% F_leg_x       = F_F10(:,1) + F_F11(:,1) + F_F12(:,1) + F_F13(:,1);
% F_leg_y       = F_F10(:,2) + F_F11(:,2) + F_F12(:,2) + F_F13(:,2);
% F_leg         = [F_leg_x  F_leg_y];
% 
% T_leg_around_B = Vec_B_P_leg(:,1) .* F_leg(:,2) - Vec_B_P_leg(:,2) .* F_leg(:,1);
% 
% D_B_P_leg  = sqrt(  Vec_B_P_leg(:,1) .^2  + Vec_B_P_leg(:,2) .^2 );
% 
% D_A_B      = sqrt(  (Pos_B(:,1) - Pos_A(:,1)) .^2  + (Pos_B(:,2) - Pos_A(:,2)) .^2 );

%%
function [B] = remove_off(A,offset_a,offset_b)
  offset     = sum(A(offset_a:offset_b,:))/(offset_b-offset_a+1);
  samples    = ones(size(A));
  offsetMat  = samples .* offset ;
  B          = A - offsetMat ;
end

function [C] = fitting_fun_1(A,B)  %% A = X ; B = COEFFI

 C  = B(1) * A.^8 + B(2) * A.^7 + B(3) * A.^6 + B(4) * A.^5 +...
      B(5) * A.^4 + B(6) * A.^3 + B(7) * A.^2 + B(8) * A.^1 + B(9) ;
end


function [C] = fit_fun_gauss(A,B)  %% A = X ; B = COEFFI
      order = length(B)/3;
      C = zeros(size(A)) ;
      for i = 1:order
          C = B(3*i-2) .* exp(-((A - B(3*i-1))./ B(3*i)) .^2) + C ;
      end

end

function [C] = fit_fun_power(A,B)  %% A = X ; B = COEFFI
    
          C = B(1)*A.^B(2)+ B(3);
      

end


function [C] = fit_fun_log(A,B)  %% A = X ; B = COEFFI
    
          C = B(1)./(1 + exp(-B(2) .* (A - B(3)) ));
      

end



function [C] = fit_fun_log_4(A,B)  %% A = X ; B = COEFFI
    
          C = B(4) + (B(1)-B(4))/(1 + (A./B(3)).^B(2));
      

end

function [K] = fit_gauss_3(A,B)
  
    [xData, yData] = prepareCurveData( A, B);

    % Set up fittype and options.
    ft = fittype( 'gauss3' );
    opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
    opts.Display = 'Off';
    
    % Fit model to data.
    [fitresult, gof] = fit( xData, yData, ft, opts );

    K   =  [fitresult.a1 fitresult.b1 fitresult.c1 fitresult.a2 fitresult.b2 fitresult.c2 fitresult.a3 fitresult.b3 fitresult.c3 ];
end

%% icc function


function [r, LB, UB, F, df1, df2, p] = ICC(M, type, alpha, r0)
% Intraclass correlation
%   [r, LB, UB, F, df1, df2, p] = ICC(M, type, alpha, r0)
%
%   M is matrix of observations. Each row is an object of measurement and
%   each column is a judge or measurement.
%
%   'type' is a string that can be one of the six possible codes for the desired
%   type of ICC:
%       '1-1': The degree of absolute agreement among measurements made on
%         randomly seleted objects. It estimates the correlation of any two
%         measurements.
%       '1-k': The degree of absolute agreement of measurements that are
%         averages of k independent measurements on randomly selected
%         objects.
%       'C-1': case 2: The degree of consistency among measurements. Also known
%         as norm-referenced reliability and as Winer's adjustment for
%         anchor points. case 3: The degree of consistency among measurements maded under
%         the fixed levels of the column factor. This ICC estimates the
%         corrlation of any two measurements, but when interaction is
%         present, it underestimates reliability.
%       'C-k': case 2: The degree of consistency for measurements that are
%         averages of k independent measurements on randomly selected
%         onbjectgs. Known as Cronbach's alpha in psychometrics. case 3:  
%         The degree of consistency for averages of k independent
%         measures made under the fixed levels of column factor.
%       'A-1': case 2: The degree of absolute agreement among measurements. Also
%         known as criterion-referenced reliability. case 3: The absolute 
%         agreement of measurements made under the fixed levels of the column factor.
%       'A-k': case 2: The degree of absolute agreement for measurements that are
%         averages of k independent measurements on randomly selected objects.
%         case 3: he degree of absolute agreement for measurements that are
%         based on k independent measurements maded under the fixed levels
%         of the column factor.
%
%       ICC is the estimated intraclass correlation. LB and UB are upper
%       and lower bounds of the ICC with alpha level of significance. 
%
%       In addition to estimation of ICC, a hypothesis test is performed
%       with the null hypothesis that ICC = r0. The F value, degrees of
%       freedom and the corresponding p-value of the this test are
%       reported.
%
%       (c) Arash Salarian, 2008
%
%       Reference: McGraw, K. O., Wong, S. P., "Forming Inferences About
%       Some Intraclass Correlation Coefficients", Psychological Methods,
%       Vol. 1, No. 1, pp. 30-46, 1996
%

if nargin < 3
    alpha = .05;
end

if nargin < 4
    r0 = 0;
end

[n, k] = size(M);

SStotal = var(M(:)) *(n*k - 1);
MSR = var(mean(M, 2)) * k;
MSW = sum(var(M,0, 2)) / n;
MSC = var(mean(M, 1)) * n;
MSE = (SStotal - MSR *(n - 1) - MSC * (k -1))/ ((n - 1) * (k - 1));

switch type
    case '1-1'
        [r, LB, UB, F, df1, df2, p] = ICC_case_1_1(MSR, MSE, MSC, MSW, alpha, r0, n, k);
    case '1-k'
        [r, LB, UB, F, df1, df2, p] = ICC_case_1_k(MSR, MSE, MSC, MSW, alpha, r0, n, k);
    case 'C-1'
        [r, LB, UB, F, df1, df2, p] = ICC_case_C_1(MSR, MSE, MSC, MSW, alpha, r0, n, k);
    case 'C-k'
        [r, LB, UB, F, df1, df2, p] = ICC_case_C_k(MSR, MSE, MSC, MSW, alpha, r0, n, k);
    case 'A-1'
        [r, LB, UB, F, df1, df2, p] = ICC_case_A_1(MSR, MSE, MSC, MSW, alpha, r0, n, k);
    case 'A-k'
        [r, LB, UB, F, df1, df2, p] = ICC_case_A_k(MSR, MSE, MSC, MSW, alpha, r0, n, k);
end


%----------------------------------------
function [r, LB, UB, F, df1, df2, p] = ICC_case_1_1(MSR, MSE, MSC, MSW, alpha, r0, n, k)
r = (MSR - MSW) / (MSR + (k-1)*MSW);

F = (MSR/MSW) * (1-r0)/(1+(k-1)*r0);
df1 = n-1;
df2 = n*(k-1);
p = 1-fcdf(F, df1, df2);

FL = (MSR/MSW) / finv(1-alpha/2, n-1, n*(k-1));
FU = (MSR/MSW) * finv(1-alpha/2, n*(k-1), n-1);

LB = (FL - 1) / (FL + (k-1));
UB = (FU - 1) / (FU + (k-1));
end
%----------------------------------------
function [r, LB, UB, F, df1, df2, p] = ICC_case_1_k(MSR, MSE, MSC, MSW, alpha, r0, n, k)
r = (MSR - MSW) / MSR;

F = (MSR/MSW) * (1-r0);
df1 = n-1;
df2 = n*(k-1);
p = 1-fcdf(F, df1, df2);

FL = (MSR/MSW) / finv(1-alpha/2, n-1, n*(k-1));
FU = (MSR/MSW) * finv(1-alpha/2, n*(k-1), n-1);

LB = 1 - 1 / FL;
UB = 1 - 1 / FU;
end
%----------------------------------------
function [r, LB, UB, F, df1, df2, p] = ICC_case_C_1(MSR, MSE, MSC, MSW, alpha, r0, n, k)
r = (MSR - MSE) / (MSR + (k-1)*MSE);

F = (MSR/MSE) * (1-r0)/(1+(k-1)*r0);
df1 = n - 1;
df2 = (n-1)*(k-1);
p = 1-fcdf(F, df1, df2);

FL = (MSR/MSE) / finv(1-alpha/2, n-1, (n-1)*(k-1));
FU = (MSR/MSE) * finv(1-alpha/2, (n-1)*(k-1), n-1);

LB = (FL - 1) / (FL + (k-1));
UB = (FU - 1) / (FU + (k-1));
end
%----------------------------------------
function [r, LB, UB, F, df1, df2, p] = ICC_case_C_k(MSR, MSE, MSC, MSW, alpha, r0, n, k)
r = (MSR - MSE) / MSR;

F = (MSR/MSE) * (1-r0);
df1 = n - 1;
df2 = (n-1)*(k-1);
p = 1-fcdf(F, df1, df2);

FL = (MSR/MSE) / finv(1-alpha/2, n-1, (n-1)*(k-1));
FU = (MSR/MSE) * finv(1-alpha/2, (n-1)*(k-1), n-1);

LB = 1 - 1 / FL;
UB = 1 - 1 / FU;
end
%----------------------------------------
function [r, LB, UB, F, df1, df2, p] = ICC_case_A_1(MSR, MSE, MSC, MSW, alpha, r0, n, k)
r = (MSR - MSE) / (MSR + (k-1)*MSE + k*(MSC-MSE)/n);

a = (k*r0) / (n*(1-r0));
b = 1 + (k*r0*(n-1))/(n*(1-r0));
F = MSR / (a*MSC + b*MSE);
%df2 = (a*MSC + b*MSE)^2/((a*MSC)^2/(k-1) + (b*MSE)^2/((n-1)*(k-1)));

a = k*r/(n*(1-r));
b = 1+k*r*(n-1)/(n*(1-r));
v = (a*MSC + b*MSE)^2/((a*MSC)^2/(k-1) + (b*MSE)^2/((n-1)*(k-1)));

df1 = n - 1;
df2 = v;
p = 1-fcdf(F, df1, df2);

Fs = finv(1-alpha/2, n-1, v);
LB = n*(MSR - Fs*MSE)/(Fs*(k*MSC + (k*n - k - n)*MSE) + n*MSR);

Fs = finv(1-alpha/2, v, n-1);
UB = n*(Fs*MSR-MSE)/(k*MSC + (k*n - k - n)*MSE + n*Fs*MSR);
end
%----------------------------------------
function [r, LB, UB, F, df1, df2, p] = ICC_case_A_k(MSR, MSE, MSC, MSW, alpha, r0, n, k)
r = (MSR - MSE) / (MSR + (MSC-MSE)/n);

c = r0/(n*(1-r0));
d = 1 + (r0*(n-1))/(n*(1-r0));
F = MSR / (c*MSC + d*MSE);
%df2 = (c*MSC + d*MSE)^2/((c*MSC)^2/(k-1) + (d*MSE)^2/((n-1)*(k-1)));

a = k*r/(n*(1-r));
b = 1+k*r*(n-1)/(n*(1-r));
v = (a*MSC + b*MSE)^2/((a*MSC)^2/(k-1) + (b*MSE)^2/((n-1)*(k-1)));

df1 = n - 1;
df2 = v;
p = 1-fcdf(F, df1, df2);

Fs = finv(1-alpha/2, n-1, v);
LB = n*(MSR - Fs*MSE)/(Fs*(MSC-MSE) + n*MSR);

Fs = finv(1-alpha/2, v, n-1);
UB = n*(Fs*MSR - MSE)/(MSC - MSE + n*Fs*MSR);
end
end