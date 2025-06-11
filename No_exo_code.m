clear all; close all; clc
smoothdata_window = 100;
run No_exo_force.m ;
run No_exo_angle.m ;
%%
load("No_exo_force_chest_offset.mat");
load("No_exo_force_hip_offset.mat");
load("No_exo_force_leg_offset.mat");
load("no_exo_angle_degree_bending.mat");
load("no_exo_angle_degree_lower_back_joint.mat");

%% prepare fitting

no_exo_hip_hori_offset     = No_exo_force_hip_offset.no_exo_hip_hori_offset ;
no_exo_hip_verti_offset    = No_exo_force_hip_offset.no_exo_hip_verti_offset ;
no_exo_leg_hori_top_offset = No_exo_force_leg_offset.no_exo_leg_hori_top_offset;
no_exo_leg_hori_bot_offset = No_exo_force_leg_offset.no_exo_leg_hori_bot_offset;
no_exo_leg_verti_offset    = No_exo_force_leg_offset.no_exo_leg_verti_offset;
no_exo_leg_verti_front_offset    = No_exo_force_leg_offset.no_exo_leg_verti_front;
no_exo_leg_verti_back_offset     = No_exo_force_leg_offset.no_exo_leg_verti_back;
no_exo_hip_hori_top_offset       = No_exo_force_hip_offset.no_exo_hip_hori_top_offset;
no_exo_hip_hori_bot_offset       = No_exo_force_hip_offset.no_exo_hip_hori_bot_offset;
no_exo_hip_verti_front_offset       = No_exo_force_hip_offset.no_exo_hip_verti_front_offset;
no_exo_hip_verti_back_offset        = No_exo_force_hip_offset.no_exo_hip_verti_back_offset;

fit_xData = [angle_degree_lower_back_joint,  angle_degree_lower_back_joint,  angle_degree_lower_back_joint, ...
             angle_degree_lower_back_joint,   angle_degree_lower_back_joint,   angle_degree_lower_back_joint, ...
             angle_degree_bending,   angle_degree_bending,   angle_degree_bending, ...
             angle_degree_bending,   angle_degree_bending ];

fit_yData = [no_exo_hip_hori_offset, no_exo_hip_verti_offset, no_exo_hip_hori_top_offset, ...
             no_exo_hip_hori_bot_offset, no_exo_hip_verti_front_offset, no_exo_hip_verti_back_offset,...
             no_exo_leg_hori_top_offset,  no_exo_leg_hori_bot_offset,...
             no_exo_leg_verti_offset,     no_exo_leg_verti_front_offset, no_exo_leg_verti_back_offset  ];

%   function [fitresult, gof] = createFit(angle_degree_lower_back_joint, no_exo_sum_hip_hori);

Fit_force = zeros(size(angle_degree_lower_back_joint));
%%
x_label = {'angle of hip','angle of hip','angle of hip','angle of hip','angle of hip','angle of hip', ....
           'angle of leg','angle of leg','angle of leg','angle of leg','angle of leg'};
y_label = {'hip hori','hip verti', 'hip hori top','hip hori bot','hip verti front','hip verti back'...
           'leg hori top', 'leg hori bot', 'leg verti','leg verti front','leg verti back'};

f1 = figure();  
f2 = figure();  
fit_coefficient = zeros(2,11);
gg = zeros(length(fit_xData(:,1)),11);




for i = 1:11

   if i == 1 || i == 3 || i == 4 || i == 7 || i == 8

   FF = fit_hori(fit_xData(:,i), fit_yData(:,i));
   fit_coefficient(1,i) = FF.a;  
   fit_coefficient(2,i) = FF.b;

   gg(:,i) = fit_coefficient(1,i) * sin(fit_xData(:,i)/360*2*pi) + fit_coefficient(2,i) ;

   end

   if i == 2 || i == 5 || i == 6 || i == 9 || i == 10 || i == 11

   FF = fit_verti(fit_xData(:,i), fit_yData(:,i));
   fit_coefficient(1,i) = FF.a;  
   fit_coefficient(2,i) = FF.b;
   gg(:,i) = fit_coefficient(1,i) * cos(fit_xData(:,i)/360*2*pi) + fit_coefficient(2,i) ;
   end


    title('Fitting between force and angle')  
    if i<= 6   
        figure(f1);
        subplot(2,3,i)
        plot(fit_xData(:,i), fit_yData(:,i),'b');
        hold on;
        plot(fit_xData(:,i), gg(:,i),'r');
        grid on;
        ax = gca;
        t  = ax.Title;
        x  = ax.XLabel;
        y  = ax.YLabel;
        set(t,"String",y_label{i});
        set(x,"String",'angle');
        set(y,"String",'force');
    end
    if i >= 7           
        figure(f2);
        subplot(5,1,i-6)
        plot(fit_xData(:,i), fit_yData(:,i),'b');
        hold on;
        plot(fit_xData(:,i), gg(:,i),'r');
        grid on;
        ax = gca;
        t  = ax.Title;
        x  = ax.XLabel;
        y  = ax.YLabel;
        set(t,"String",y_label{i});
        set(x,"String",'angle');
        set(y,"String",'force');
    end
end


% No_exo_chest_hori_front_top = mean(no_exo_chest_hori_front_top_offset);
% No_exo_chest_hori_front_bot = mean(no_exo_chest_hori_front_bot_offset);
% No_exo_chest_hori_back      = mean(no_exo_chest_hori_back_offset);
% No_exo_chest_verti = mean(no_exo_chest_verti_offset);


% save('No_exo_chest_hori_front_top','No_exo_chest_hori_front_top');
% save('No_exo_chest_hori_front_bot','No_exo_chest_hori_front_bot');
% save('No_exo_chest_hori_back','No_exo_chest_hori_back');
% save('No_exo_chest_verti','No_exo_chest_verti');


save('fit_coefficient.mat','fit_coefficient');

%%






%%


function [C] = fitting_fun(A,B)         %% A fitting coefficient got from no-exo code
  C = A(1)*B.^5 + A(2)*B.^4 + A(3)*B.^3 + A(4)*B.^2 + A(5)*B.^1+ A(6)*B .^0 ;  %% B angle got from exo condition
    
end


function [C] = fit_hori(A,B)  %% A = angle, B = force

[xData, yData] = prepareCurveData(A,B);

% Set up fittype and options.
ft = fittype( 'a*sin(x/360*2*pi)+b', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.Robust = 'Bisquare';

% Fit model to data.
[C, gof] = fit( xData, yData, ft, opts );


end


function [C] = fit_verti(A,B)  %% A = angle, B = force

[xData, yData] = prepareCurveData( A,B);

% Set up fittype and options.
ft = fittype( 'a*cos(x/360*2*pi)+b', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.Robust = 'Bisquare';

% Fit model to data.
[C, gof] = fit( xData, yData, ft, opts );


end