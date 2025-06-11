%% parameters input
% clc;clear;close all;
offset_a = 100;
offset_b = 1000;
smootgdata_window = 100 ;
% load('coefficient.mat');
K_LC = readtable('C:\Users\Haibing\OneDrive\OneDrive - Vrije Universiteit Brussel\Desktop\New_folder\new_code\load_cell_cali\k_of_load_cell.xlsx');
LC_reverse = [6 9 11 12 18 28];
K_modi     = K_LC.K ;
K_modi(LC_reverse) = -1 .*   K_modi(LC_reverse);
%% vicon signal reading
addpath('C:\Program Files\Vicon\Nexus2.14\SDK\MATLAB');
vicon = ViconNexus();

% cameraRate = vicon.GetFrameRate(); %Hz
% [boxX, boxY, boxZ, valid]=vicon.GetTrajectory('tesdtoll','fixed1');

deviceIDs = vicon.GetDeviceIDs();
[~,~,~,outputIDs,~,~] = vicon.GetDeviceDetails(deviceIDs(1));
[~,~,~,~,~,channelIDs] = vicon.GetDeviceOutputDetails(deviceIDs(1),outputIDs(1));
[dum, ~, analogueRate] = vicon.GetDeviceChannel(deviceIDs(1),outputIDs(1),channelIDs(1));

data = zeros(28,length(dum));
time = (1:length(dum))/analogueRate;

for i = 1 :28
[data(i,:), ~, ~] = vicon.GetDeviceChannel(deviceIDs(1),outputIDs(1),channelIDs(i));
end

reading_data =data'; 

%% FORCE DATA (Newton)

exo_force_data_raw       = K_modi' .* reading_data + K_LC.b';
exo_force_data_sm        = smoothdata(exo_force_data_raw,'rlowess',smootgdata_window); 
exo_force_data_offset    = 9.8 .* exo_force_data_sm;

exo_force_data_no_offset = exo_force_data_sm -  mean(exo_force_data_sm(offset_a:offset_b,:));
force                = 9.8 .* exo_force_data_no_offset;

%% chest
chest_hori_front_left_bot   = exo_force_data_offset(:,14);
chest_hori_front_left_top   = exo_force_data_offset(:,15);
chest_hori_front_right_bot  = exo_force_data_offset(:,24);
chest_hori_front_right_top  = exo_force_data_offset(:,25);

chest_hori_back_left        = exo_force_data_offset(:,18);
chest_hori_back_right       = exo_force_data_offset(:,28);

chest_verti_front_left      = exo_force_data_offset(:,16);
chest_verti_back_left       = exo_force_data_offset(:,17);
chest_verti_front_right     = exo_force_data_offset(:,26);
chest_verti_back_right      = exo_force_data_offset(:,27);

chest_hori_front_top  = chest_hori_front_left_top +  chest_hori_front_right_top ;
chest_hori_front_bot  = chest_hori_front_left_bot +  chest_hori_front_right_bot ;
chest_hori_back       = chest_hori_back_left      +  chest_hori_back_right;
chest_hori            = chest_hori_front_top + chest_hori_front_bot + chest_hori_back;

chest_hori_left       = chest_hori_front_left_top + chest_hori_front_left_bot + chest_hori_back_left;
chest_hori_right      = chest_hori_front_right_top + chest_hori_front_right_bot + chest_hori_back_right;

chest_verti_front     = chest_verti_front_left + chest_verti_front_right;
chest_verti_back      = chest_verti_back_left  + chest_verti_back_right;
chest_verti           = chest_verti_front + chest_verti_back;

chest_verti_left      = chest_verti_front_left + chest_verti_back_left ; 
chest_verti_right     = chest_verti_front_right + chest_verti_back_right ; 

field1  = 'chest_hori_front_left_bot'                ;  value1 = chest_hori_front_left_bot;
field2  = 'chest_hori_front_left_top'                ;  value2 = chest_hori_front_left_top;
field3  = 'chest_hori_front_right_bot'                ; value3 = chest_hori_front_right_bot;
field4  = 'chest_hori_front_right_top'                ; value4 = chest_hori_front_right_top;
field5  = 'chest_hori_back_left'                ;       value5 = chest_hori_back_left;
field6  = 'chest_hori_back_right'                ;      value6 = chest_hori_back_right;
field7  = 'chest_verti_front_left'                ;     value7 = chest_verti_front_left;
field8  = 'chest_verti_back_left'                ;      value8 = chest_verti_back_left;
field9  = 'chest_verti_front_right'                ;    value9 = chest_verti_front_right;
field10 = 'chest_verti_back_right'                ;    value10 = chest_verti_back_right ;
field11 = 'chest_hori_front_top'                ;      value11 = chest_hori_front_top;
field12 = 'chest_hori_front_bot'                ;      value12 = chest_hori_front_bot;
field13 = 'chest_hori_back'                ;           value13 = chest_hori_back;
field14 = 'chest_hori'                ;                value14 = chest_hori;
field15 = 'chest_hori_left'                ;           value15 = chest_hori_left;
field16 = 'chest_hori_right'                ;          value16 = chest_hori_right;
field17 = 'chest_verti_front'                ;         value17 = chest_verti_front;
field18 = 'chest_verti_back'                ;          value18 = chest_verti_back;
field19 = 'chest_verti'                ;               value19 = chest_verti;
field20 = 'chest_verti_left'                ;          value20 = chest_verti_left;
field21 = 'chest_verti_right'                ;         value21 = chest_verti_right;




exo_force_chest_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8,field9,value9,...
                                field10,value10,field11,value11,field12,value12,field13,value13,field14,value14,field15,value15,field16,value16,field17,value17...
                                ,field18,value18,field19,value19,field20,value20,field21,value21);
save('exo_force_chest_offset',"exo_force_chest_offset");
%% hip
hip_hori_left   = exo_force_data_offset(:,8) + exo_force_data_offset(:,9);
hip_hori_right  = exo_force_data_offset(:,12) + exo_force_data_offset(:,13);
hip_verti_left  = exo_force_data_offset(:,6) + exo_force_data_offset(:,7);
hip_verti_right = exo_force_data_offset(:,10) + exo_force_data_offset(:,11);
hip_hori_top    = exo_force_data_offset(:,8)  + exo_force_data_offset(:,12);
hip_hori_bot    = exo_force_data_offset(:,9)  + exo_force_data_offset(:,13);
hip_verti_front = exo_force_data_offset(:,6)  + exo_force_data_offset(:,10);
hip_verti_back  = exo_force_data_offset(:,7)  + exo_force_data_offset(:,11);

hip_hori    = (exo_force_data_offset(:,8) + exo_force_data_offset(:,9) + exo_force_data_offset(:,12) + exo_force_data_offset(:,13));
hip_verti   = (exo_force_data_offset(:,6) + exo_force_data_offset(:,7) + exo_force_data_offset(:,10) + exo_force_data_offset(:,11));

field1  = 'hip_hori_left'                ;  value1 = hip_hori_left;
field2  = 'hip_hori_right'                ;  value2 = hip_hori_right;
field3  = 'hip_verti_left'                ; value3 = hip_verti_left;
field4  = 'hip_verti_right'                ; value4 = hip_verti_right     ;
field5  = 'hip_hori_top'                ;       value5 = hip_hori_top;
field6  = 'hip_hori_bot'                ;      value6 = hip_hori_bot;
field7  = 'hip_verti_front'                ;     value7 = hip_verti_front;
field8  = 'hip_verti_back'                ;      value8 = hip_verti_back;
field9  = 'hip_hori'                ;    value9 = hip_hori;
field10 = 'hip_verti'                ;    value10 = hip_verti ;

exo_force_hip_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8,field9,value9,...
                                field10,value10);
save('exo_force_hip_offset',"exo_force_hip_offset");


% field1 = 'hip_hori';   value1 = hip_hori;
% field2 = 'hip_verti';  value2 = hip_verti;
% hip = struct(field1,value1,field2,value2);

%% leg

leg_hori_top_left   = exo_force_data_offset(:,4);
leg_hori_top_right  = exo_force_data_offset(:,22);
leg_hori_bot_left   = exo_force_data_offset(:,3) + exo_force_data_offset(:,5);
leg_hori_bot_right  = exo_force_data_offset(:,21) +exo_force_data_offset(:,23);

leg_verti_left      = exo_force_data_offset(:,1) + exo_force_data_offset(:,2);
leg_verti_right     = exo_force_data_offset(:,19) +exo_force_data_offset(:,20);


leg_hori_top = (exo_force_data_offset(:,4) + exo_force_data_offset(:,22));
leg_hori_bot = (exo_force_data_offset(:,3) + exo_force_data_offset(:,5) + exo_force_data_offset(:,21) +exo_force_data_offset(:,23)) ;
leg_hori     = leg_hori_top + leg_hori_bot;
leg_verti    = (exo_force_data_offset(:,1) + exo_force_data_offset(:,2) + exo_force_data_offset(:,19) +exo_force_data_offset(:,20));
leg_verti_front = exo_force_data_offset(:,2) + exo_force_data_offset(:,19);
leg_verti_back  = exo_force_data_offset(:,1) + exo_force_data_offset(:,20);

field1  = 'leg_hori_top_left'                ;  value1 = leg_hori_top_left;
field2  = 'leg_hori_top_right'                ;  value2 = leg_hori_top_right;
field3  = 'leg_hori_bot_left'                ; value3 = leg_hori_bot_left;
field4  = 'leg_hori_bot_right'                ; value4 = leg_hori_bot_right     ;
field5  = 'leg_verti_left'                ;       value5 = leg_verti_left;
field6  = 'leg_verti_right'                ;      value6 = leg_verti_right ;
field7  = 'leg_hori_top'                ;     value7 = leg_hori_top;
field8  = 'leg_hori_bot'                ;      value8 = leg_hori_bot;
field9  = 'leg_hori'                ;    value9 = leg_hori;
field10 = 'leg_verti'                ;    value10 = leg_verti ;
field11 = 'leg_verti_front'          ;    value11 = exo_force_data_offset(:,2) + exo_force_data_offset(:,19);
field12 = 'leg_verti_back'          ;    value12 = exo_force_data_offset(:,1) + exo_force_data_offset(:,20);


exo_force_leg_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8,field9,value9,...
                                field10,value10,field11,value11,field12,value12);
save('exo_force_leg_offset',"exo_force_leg_offset");


% field1 = 'leg_hori_top';  value1 = leg_hori_top;
% field2 = 'leg_hori_bot';  value2 = leg_hori_bot;
% field3 = 'leg_verti';     value3 = leg_verti;
% leg = struct(field1,value1,field2,value2,field3,value3);
% 
% Force.chest = chest;
% Force.hip   = hip;
% Force.leg   = leg;


%% figure

figure(); 
t = tiledlayout(3,1,'TileSpacing','Compact');

ax1 = nexttile;
plot(time,chest_hori_front_top);grid on; title('Force Distribution on the Chest'); 
% xlabel('Time(s)'); ylabel('Force(N)');
hold on; 
plot(time,chest_hori_front_bot); 
plot(time,chest_hori_back); 
plot(time,chest_verti);    
legend('Chest Hori Front Top','Chest Hori Front Bot','Chest Hori Back', 'Chest Verti');
hold off;

ax2 = nexttile;
plot(time,hip_hori);grid on;       
hold on; 
plot(time,hip_verti); title('Force Distribution on the Hip'); 
% xlabel('Time(s)'); ylabel('Force(N)');
legend('Hip Hori','Hip Verti');
hold off;

ax3 = nexttile;
plot(time,leg_hori_top);grid on;   
hold on; 
plot(time,leg_hori_bot);   
plot(time,leg_verti); 
title('Force Distribution on the Legs'); 
% xlabel('Time(s)'); ylabel('Force(N)');
legend('Leg Hori Top','Leg Hori Bot','Leg Verti');
hold off;

sgtitle(t,'Force Distribution on the Diverse Segements')

xlabel(t,'Time(s)'); ylabel(t,'Force(N)');
linkaxes([ax1 ax2 ax3],'x');
ax1.XLim = [0 max(time)+0.2];







