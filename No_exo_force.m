%% parameter
% clc;clear;close all;
offset_a = 100;
offset_b = 500;
smoothdata_window = 100;

%%
% load('coefficient.mat');
K_LC = readtable('C:\Users\Haibing\OneDrive\OneDrive - Vrije Universiteit Brussel\Desktop\New_folder\new_code\load_cell_cali\k_of_load_cell.xlsx');



LC_reverse = [6 9 11 12 18 28];
K_modi     = K_LC.K ;
K_modi(LC_reverse) = -1 .*   K_modi(LC_reverse);



%% vicon data
vicon = ViconNexus();
% cameraRate = vicon.GetFrameRate(); %Hz
% [boxX, boxY, boxZ, valid]=vicon.GetTrajectory('tesdtoll','fixed1');
Subject = vicon.GetSubjectNames();   %%%% exo no-exo
cameraRate = vicon.GetFrameRate();   %Hz

markers = vicon.GetMarkerNames( Subject{1} ); %% name of markers
Joints  = vicon.GetJointNames( Subject{1} );
deviceIDs = vicon.GetDeviceIDs();
[~,~,~,outputIDs,~,~] = vicon.GetDeviceDetails(deviceIDs(1));
[~,~,~,~,~,channelIDs] = vicon.GetDeviceOutputDetails(deviceIDs(1),outputIDs(1));
[dum, ~, analogueRate] = vicon.GetDeviceChannel(deviceIDs(1),outputIDs(1),channelIDs(1));

N_LC = length(channelIDs);

data = zeros(N_LC,length(dum));
time = (1:length(dum))/analogueRate;

for i = 1 :N_LC
[data(i,:), ~, ~] = vicon.GetDeviceChannel(deviceIDs(1),outputIDs(1),channelIDs(i));
end

reading_data =data'; 
%% FORCE DATA

force_data_raw       = K_modi' .* reading_data + K_LC.b';
force_data_sm        = smoothdata(force_data_raw,'rlowess',smoothdata_window); 
no_exo_force_offset  = 9.8 .* force_data_sm;

force_data_no_offset = remove_off(force_data_sm,offset_a,offset_b);
no_exo_force_no_offset                = 9.8 .* force_data_no_offset;

%% chest

no_exo_chest_hori_front_left_bot_offset   = no_exo_force_offset(:,14);
no_exo_chest_hori_front_left_top_offset   = no_exo_force_offset(:,15);
no_exo_chest_hori_front_right_bot_offset  = no_exo_force_offset(:,24);
no_exo_chest_hori_front_right_top_offset  = no_exo_force_offset(:,25);

no_exo_chest_hori_back_left_offset        = no_exo_force_offset(:,18);
no_exo_chest_hori_back_right_offset       = no_exo_force_offset(:,28);

no_exo_chest_verti_front_left_offset      = no_exo_force_offset(:,16);
no_exo_chest_verti_back_left_offset       = no_exo_force_offset(:,17);
no_exo_chest_verti_front_right_offset     = no_exo_force_offset(:,26);
no_exo_chest_verti_back_right_offset      = no_exo_force_offset(:,27);

no_exo_chest_hori_front_top_offset  = no_exo_chest_hori_front_left_top_offset +  no_exo_chest_hori_front_right_top_offset ;
no_exo_chest_hori_front_bot_offset  = no_exo_chest_hori_front_left_bot_offset +  no_exo_chest_hori_front_right_bot_offset ;
no_exo_chest_hori_back_offset       = no_exo_chest_hori_back_left_offset      +  no_exo_chest_hori_back_right_offset;
no_exo_chest_hori_offset            = no_exo_chest_hori_front_top_offset + no_exo_chest_hori_front_bot_offset + no_exo_chest_hori_back_offset;

no_exo_chest_hori_left_offset       = no_exo_chest_hori_front_left_top_offset  + no_exo_chest_hori_front_left_bot_offset  + no_exo_chest_hori_back_left_offset;
no_exo_chest_hori_right_offset      = no_exo_chest_hori_front_right_top_offset + no_exo_chest_hori_front_right_bot_offset + no_exo_chest_hori_back_right_offset;


no_exo_chest_verti_front_offset     = no_exo_chest_verti_front_left_offset + no_exo_chest_verti_front_right_offset;
no_exo_chest_verti_back_offset      = no_exo_chest_verti_back_left_offset  + no_exo_chest_verti_back_right_offset;
no_exo_chest_verti_offset           = no_exo_chest_verti_front_offset + no_exo_chest_verti_back_offset;

no_exo_chest_verti_left_offset       = no_exo_chest_verti_front_left_offset + no_exo_chest_verti_back_left_offset ; 
no_exo_chest_verti_right_offset      = no_exo_chest_verti_front_right_offset + no_exo_chest_verti_back_right_offset ; 

field1 = 'no_exo_chest_hori_front_left_bot_offset';      value1 = no_exo_chest_hori_front_left_bot_offset;
field2 = 'no_exo_chest_hori_front_left_top_offset';      value2 = no_exo_chest_hori_front_left_top_offset;
field3 = 'no_exo_chest_hori_front_right_bot_offset';     value3 = no_exo_chest_hori_front_right_bot_offset;
field4 = 'no_exo_chest_hori_front_right_top_offset';     value4 = no_exo_chest_hori_front_right_top_offset;
field5 = 'no_exo_chest_hori_back_left_offset';           value5 = no_exo_chest_hori_back_left_offset;
field6 = 'no_exo_chest_hori_back_right_offset';          value6 = no_exo_chest_hori_back_right_offset;
field7 = 'no_exo_chest_verti_front_left_offset';         value7 = no_exo_chest_verti_front_left_offset;
field8 = 'no_exo_chest_verti_back_left_offset';          value8 = no_exo_chest_verti_back_left_offset;
field9 = 'no_exo_chest_verti_front_right_offset';        value9 = no_exo_chest_verti_front_right_offset;
field10 = 'no_exo_chest_verti_back_right_offset';        value10 = no_exo_chest_verti_back_right_offset;
field11 = 'no_exo_chest_verti_offset';                   value11 = no_exo_chest_verti_offset;
field12 = 'no_exo_chest_hori_offset';                    value12 = no_exo_chest_hori_offset;

No_exo_force_chest_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
                             field6,value6,field7,value7,field8,value8,field9,value9,field10,value10,field11,value11,field12,value12);

save('No_exo_force_chest_offset',"No_exo_force_chest_offset");
% no_exo_chest_hori_front_left_bot_no_offset   = no_exo_force_no_offset(:,14);
% no_exo_chest_hori_front_left_top_no_offset   = no_exo_force_no_offset(:,15);
% no_exo_chest_hori_front_right_bot_no_offset  = no_exo_force_no_offset(:,24);
% no_exo_chest_hori_front_right_top_no_offset  = no_exo_force_no_offset(:,25);
% 
% no_exo_chest_hori_back_left_no_offset        = no_exo_force_no_offset(:,18);
% no_exo_chest_hori_back_right_no_offset       = no_exo_force_no_offset(:,28);
% 
% no_exo_chest_verti_front_left_no_offset      = no_exo_force_no_offset(:,16);
% no_exo_chest_verti_back_left_no_offset       = no_exo_force_no_offset(:,17);
% no_exo_chest_verti_front_right_no_offset     = no_exo_force_no_offset(:,26);
% no_exo_chest_verti_back_right_no_offset      = no_exo_force_no_offset(:,27);
% 
% no_exo_chest_hori_front_top_no_offset  = no_exo_chest_hori_front_left_top_no_offset +  no_exo_chest_hori_front_right_top_no_offset ;
% no_exo_chest_hori_front_bot_no_offset  = no_exo_chest_hori_front_left_bot_no_offset +  no_exo_chest_hori_front_right_bot_no_offset ;
% no_exo_chest_hori_back_no_offset       = no_exo_chest_hori_back_left_no_offset      +  no_exo_chest_hori_back_right_no_offset;
% no_exo_chest_hori_no_offset            = no_exo_chest_hori_front_top_no_offset + no_exo_chest_hori_front_bot_no_offset + no_exo_chest_hori_back_no_offset;
% 
% no_exo_chest_hori_left_no_offset       = no_exo_chest_hori_front_left_top_no_offset  + no_exo_chest_hori_front_left_bot_no_offset  + no_exo_chest_hori_back_left_no_offset;
% no_exo_chest_hori_right_no_offset      = no_exo_chest_hori_front_right_top_no_offset + no_exo_chest_hori_front_right_bot_no_offset + no_exo_chest_hori_back_right_no_offset;
% 
% 
% no_exo_chest_verti_front_no_offset     = no_exo_chest_verti_front_left_no_offset + no_exo_chest_verti_front_right_no_offset;
% no_exo_chest_verti_back_no_offset      = no_exo_chest_verti_back_left_no_offset  + no_exo_chest_verti_back_right_no_offset;
% no_exo_chest_verti_no_offset           = no_exo_chest_verti_front_no_offset + no_exo_chest_verti_back_no_offset;
% 
% no_exo_chest_verti_left_no_offset       = no_exo_chest_verti_front_left_no_offset + no_exo_chest_verti_back_left_no_offset ; 
% no_exo_chest_verti_right_no_offset      = no_exo_chest_verti_front_right_no_offset + no_exo_chest_verti_back_right_no_offset ; 
% 
% 
% field1 = 'no_exo_chest_hori_front_left_bot_no_offset';      value1 = no_exo_chest_hori_front_left_bot_no_offset;
% field2 = 'no_exo_chest_hori_front_left_top_no_offset';      value2 = no_exo_chest_hori_front_left_top_no_offset;
% field3 = 'no_exo_chest_hori_front_right_bot_no_offset';     value3 = no_exo_chest_hori_front_right_bot_no_offset;
% field4 = 'no_exo_chest_hori_front_right_top_no_offset';     value4 = no_exo_chest_hori_front_right_top_no_offset;
% field5 = 'no_exo_chest_hori_back_left_no_offset';           value5 = no_exo_chest_hori_back_left_no_offset;
% field6 = 'no_exo_chest_hori_back_right_no_offset';          value6 = no_exo_chest_hori_back_right_no_offset;
% field7 = 'no_exo_chest_verti_front_left_no_offset';         value7 = no_exo_chest_verti_front_left_no_offset;
% field8 = 'no_exo_chest_verti_back_left_no_offset';          value8 = no_exo_chest_verti_back_left_no_offset;
% field9 = 'no_exo_chest_verti_front_right_no_offset';        value9 = no_exo_chest_verti_front_right_no_offset;
% field10 = 'no_exo_chest_verti_back_right_no_offset';        value10 = no_exo_chest_verti_back_right_no_offset;
% 
% No_exo_chest_no_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
%                                 field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);



% no_exo_chest = figure(); 
% chest_a = tiledlayout(3,1,'TileSpacing','Compact');
% 
% chest_1 = nexttile;
% plot(time,chest_hori,'y');grid on; title('Horizontal Force Distribution on the Chest'); 
% % xlabel('Time(s)'); ylabel('Force(N)');
% hold on; 
% plot(time,chest_hori_left,'Color','red','LineWidth',0.5); 
% plot(time,chest_hori_right,'Color','red','LineWidth',0.5);    
% plot(time,chest_hori_right  + chest_hori_left,'b','LineWidth',0.5); 
% 
% legend('Chest Hori ','Chest Hori L','Chest Hori R','Chest Hori L + R');
% hold off;



%% hip

no_exo_hip_hori_left_offset   = no_exo_force_offset(:,8) + no_exo_force_offset(:,9);
no_exo_hip_hori_right_offset  = no_exo_force_offset(:,12) + no_exo_force_offset(:,13);
no_exo_hip_hori_top_offset   = no_exo_force_offset(:,8) + no_exo_force_offset(:,12);
no_exo_hip_hori_bot_offset   = no_exo_force_offset(:,9) + no_exo_force_offset(:,13);
 
no_exo_hip_verti_left_offset  = no_exo_force_offset(:,6) + no_exo_force_offset(:,7);
no_exo_hip_verti_right_offset = no_exo_force_offset(:,10) + no_exo_force_offset(:,11);
no_exo_hip_verti_front_offset  = no_exo_force_offset(:,6) + no_exo_force_offset(:,10);
no_exo_hip_verti_back_offset   = no_exo_force_offset(:,7) + no_exo_force_offset(:,11);

no_exo_hip_hori_offset    = (no_exo_force_offset(:,8) + no_exo_force_offset(:,9) + no_exo_force_offset(:,12) + no_exo_force_offset(:,13));
no_exo_hip_verti_offset   = (no_exo_force_offset(:,6) + no_exo_force_offset(:,7) + no_exo_force_offset(:,10) + no_exo_force_offset(:,11));

field1 = 'no_exo_hip_hori_left_offset';       value1  = no_exo_hip_hori_left_offset;
field2 = 'no_exo_hip_hori_right_offset';      value2  = no_exo_hip_hori_right_offset;
field3 = 'no_exo_hip_hori_top_offset';        value3  = no_exo_hip_hori_top_offset;
field4 = 'no_exo_hip_hori_bot_offset';        value4  = no_exo_hip_hori_bot_offset ;
field5 = 'no_exo_hip_verti_left_offset';      value5  = no_exo_hip_verti_left_offset;
field6 = 'no_exo_hip_verti_right_offset';     value6  = no_exo_hip_verti_right_offset;
field7 = 'no_exo_hip_verti_front_offset';     value7  = no_exo_hip_verti_front_offset;
field8 = 'no_exo_hip_verti_back_offset';      value8  = no_exo_hip_verti_back_offset;
field9 = 'no_exo_hip_hori_offset';            value9  = no_exo_hip_hori_offset;
field10 = 'no_exo_hip_verti_offset';          value10 = no_exo_hip_verti_offset;

No_exo_force_hip_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
                                field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);
save('No_exo_force_hip_offset',"No_exo_force_hip_offset");


% no_exo_hip_hori_left_no_offset    = no_exo_force_no_offset(:,8) + no_exo_force_no_offset(:,9);
% no_exo_hip_hori_right_no_offset   = no_exo_force_no_offset(:,12) + no_exo_force_no_offset(:,13);
% no_exo_hip_hori_top_no_offset     = no_exo_force_no_offset(:,8) + no_exo_force_no_offset(:,12);
% no_exo_hip_hori_bot_no_offset     = no_exo_force_no_offset(:,9) + no_exo_force_no_offset(:,13);
% 
% no_exo_hip_verti_left_no_offset   = no_exo_force_no_offset(:,6) + no_exo_force_no_offset(:,7);
% no_exo_hip_verti_right_no_offset  = no_exo_force_no_offset(:,10) + no_exo_force_no_offset(:,11);
% no_exo_hip_verti_front_no_offset  = no_exo_force_no_offset(:,6) + no_exo_force_no_offset(:,10);
% no_exo_hip_verti_back_no_offset   = no_exo_force_no_offset(:,7) + no_exo_force_no_offset(:,11);
% 
% no_exo_hip_hori_no_offset    = (no_exo_force_no_offset(:,8) + no_exo_force_no_offset(:,9) + no_exo_force_no_offset(:,12) + no_exo_force_no_offset(:,13));
% no_exo_hip_verti_no_offset   = (no_exo_force_no_offset(:,6) + no_exo_force_no_offset(:,7) + no_exo_force_no_offset(:,10) + no_exo_force_no_offset(:,11));
% 
% field1 = 'no_exo_hip_hori_left_no_offset';       value1  = no_exo_hip_hori_left_no_offset;
% field2 = 'no_exo_hip_hori_right_no_offset';      value2  = no_exo_hip_hori_right_no_offset;
% field3 = 'no_exo_hip_hori_top_no_offset';        value3  = no_exo_hip_hori_top_no_offset;
% field4 = 'no_exo_hip_hori_bot_no_offset';       value4  = no_exo_hip_hori_bot_no_offset ;
% field5 = 'no_exo_hip_verti_left_no_offset';      value5  = no_exo_hip_verti_left_no_offset;
% field6 = 'no_exo_hip_verti_right_no_offset';     value6  = no_exo_hip_verti_right_no_offset;
% field7 = 'no_exo_hip_verti_front_no_offset';     value7  = no_exo_hip_verti_front_no_offset;
% field8 = 'no_exo_hip_verti_back_no_offset';      value8  = no_exo_hip_verti_back_no_offset;
% field9 = 'no_exo_hip_hori_no_offset';            value9  = no_exo_hip_hori_no_offset;
% field10 = 'no_exo_hip_verti_no_offset';          value10 = no_exo_hip_verti_no_offset;
% 
% No_exo_hip_no_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
%                                 field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);

% field1 = 'hip_hori';   value1 = hip_hori;
% field2 = 'hip_verti';  value2 = hip_verti;
% hip = struct(field1,value1,field2,value2);

%% leg

no_exo_leg_hori_top_left_offset   = no_exo_force_offset(:,4);
no_exo_leg_hori_top_right_offset  = no_exo_force_offset(:,22);
no_exo_leg_hori_bot_left_offset   = no_exo_force_offset(:,3) + no_exo_force_offset(:,5);
no_exo_leg_hori_bot_right_offset  = no_exo_force_offset(:,21) +no_exo_force_offset(:,23);

no_exo_leg_verti_left_offset      = no_exo_force_offset(:,1) + no_exo_force_offset(:,2);
no_exo_leg_verti_right_offset     = no_exo_force_offset(:,19) +no_exo_force_offset(:,20);

no_exo_leg_hori_top_offset = (no_exo_force_offset(:,4) + no_exo_force_offset(:,22));
no_exo_leg_hori_bot_offset = (no_exo_force_offset(:,3) + no_exo_force_offset(:,5) + no_exo_force_offset(:,21) +no_exo_force_offset(:,23)) ;
no_exo_leg_verti_offset    = (no_exo_force_offset(:,1) + no_exo_force_offset(:,2) + no_exo_force_offset(:,19) +no_exo_force_offset(:,20));

field1 = 'no_exo_leg_hori_top_left_offset';        value1  = no_exo_leg_hori_top_left_offset;
field2 = 'no_exo_leg_hori_top_right_offset';       value2  = no_exo_leg_hori_top_right_offset;
field3 = 'no_exo_leg_hori_bot_left_offset';        value3  = no_exo_leg_hori_bot_left_offset;
field4 = 'no_exo_leg_hori_bot_right_offset';       value4  = no_exo_leg_hori_bot_right_offset ;
field5 = 'no_exo_leg_verti_left_offset';           value5  = no_exo_leg_verti_left_offset;
field6 = 'no_exo_leg_verti_right_offset';          value6  = no_exo_leg_verti_right_offset ;
field7 = 'no_exo_leg_hori_top_offset';             value7  = no_exo_leg_hori_top_offset;
field8 = 'no_exo_leg_hori_bot_offset';             value8  = no_exo_leg_hori_bot_offset;
field9 = 'no_exo_leg_verti_offset';                value9  = no_exo_leg_verti_offset;
field10 = 'no_exo_leg_verti_front';                value10 = no_exo_force_offset(:,2) + no_exo_force_offset(:,19);
field11 = 'no_exo_leg_verti_back';                 value11  = no_exo_force_offset(:,1) + no_exo_force_offset(:,20);


No_exo_force_leg_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
                                field6,value6,field7,value7,field8,value8,field9,value9,field10,value10,field11,value11);
save('No_exo_force_leg_offset',"No_exo_force_leg_offset");


% no_exo_leg_hori_top_left_no_offset   = no_exo_force_no_offset(:,4);
% no_exo_leg_hori_top_right_no_offset  = no_exo_force_no_offset(:,22);
% no_exo_leg_hori_bot_left_no_offset   = no_exo_force_no_offset(:,3) + no_exo_force_no_offset(:,5);
% no_exo_leg_hori_bot_right_no_offset  = no_exo_force_no_offset(:,21) +no_exo_force_no_offset(:,23);
% 
% no_exo_leg_verti_left_no_offset      = no_exo_force_no_offset(:,1) + no_exo_force_no_offset(:,2);
% no_exo_leg_verti_right_no_offset     = no_exo_force_no_offset(:,19) +no_exo_force_no_offset(:,20);
% 
% no_exo_leg_hori_top_no_offset = (no_exo_force_no_offset(:,4) + no_exo_force_no_offset(:,22));
% no_exo_leg_hori_bot_no_offset = (no_exo_force_no_offset(:,3) + no_exo_force_no_offset(:,5) + no_exo_force_no_offset(:,21) +no_exo_force_no_offset(:,23)) ;
% no_exo_leg_verti_no_offset    = (no_exo_force_no_offset(:,1) + no_exo_force_no_offset(:,2) + no_exo_force_no_offset(:,19) +no_exo_force_no_offset(:,20));
% 
% field1 = 'no_exo_leg_hori_top_left_no_offset';        value1  = no_exo_leg_hori_top_left_no_offset;
% field2 = 'no_exo_leg_hori_top_right_no_offset';       value2  = no_exo_leg_hori_top_right_no_offset;
% field3 = 'no_exo_leg_hori_bot_left_no_offset';        value3  = no_exo_leg_hori_bot_left_no_offset;
% field4 = 'no_exo_leg_hori_bot_right_no_offset';       value4  = no_exo_leg_hori_bot_right_no_offset ;
% field5 = 'no_exo_leg_verti_left_no_offset';           value5  = no_exo_leg_verti_left_no_offset;
% field6 = 'no_exo_leg_verti_right_no_offset';          value6  = no_exo_leg_verti_right_no_offset ;
% field7 = 'no_exo_leg_hori_top_no_offset';             value7  = no_exo_leg_hori_top_no_offset;
% field8 = 'no_exo_leg_hori_bot_no_offset';             value8  = no_exo_leg_hori_bot_no_offset;
% field9 = 'no_exo_leg_verti_no_offset';                value9  = no_exo_leg_verti_no_offset;
% 
% No_exo_leg_no_offset = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
%                                 field6,value6,field7,value7,field8,value8,field9,value9);


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
plot(time,no_exo_chest_hori_front_top_offset);grid on; title('Force Distribution on the Chest'); 
% xlabel('Time(s)'); ylabel('Force(N)');
hold on; 
plot(time,no_exo_chest_hori_front_bot_offset); 
plot(time,no_exo_chest_hori_back_offset); 
plot(time,no_exo_chest_verti_offset);    
legend('Chest Hori Top','Chest Hori Bot','Chest Verti');
hold off;

ax2 = nexttile;
plot(time,no_exo_hip_hori_offset);grid on;       
hold on; 
plot(time,no_exo_hip_verti_offset);      
title('Force Distribution on the Hip'); 
% xlabel('Time(s)'); ylabel('Force(N)');
legend('Hip Hori','Hip Verti');
hold off;

ax3 = nexttile;
plot(time,no_exo_leg_hori_top_offset);grid on;   
hold on; 
plot(time,no_exo_leg_hori_bot_offset);   
plot(time,no_exo_leg_verti_offset); 
title('Force Distribution on the Legs'); 
% xlabel('Time(s)'); ylabel('Force(N)');
legend('Leg Hori Top','Leg Hori Bot','Leg Verti');
hold off;

sgtitle(t,'Force Distribution on the Diverse Segements')

xlabel(t,'Time(s)'); ylabel(t,'Force(N)');
linkaxes([ax1 ax2 ax3],'x');
ax1.XLim = [0 max(time)+0.2];
%% function
function [B] = remove_off(A,offset_a,offset_b)
  offset     = sum(A(offset_a:offset_b,:))/(offset_b-offset_a+1);
  samples    = ones(size(A));
  offsetMat  = samples .* offset ;
  B          = A - offsetMat ;
end

