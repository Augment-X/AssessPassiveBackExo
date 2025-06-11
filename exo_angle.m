% clc;
% clear;close all;

%% reading Vicon data

vicon = ViconNexus();
Subject = vicon.GetSubjectNames();   %%%% exo no-exo
cameraRate = vicon.GetFrameRate();   %Hz

markers = vicon.GetMarkerNames( Subject{4} ); %% name of markers
Joints  = vicon.GetJointNames( Subject{4} );
markers_count = length(markers);
[boxX, boxY, boxZ, valid]=vicon.GetTrajectory(Subject{4},markers{1});
time    = (1:length(boxX))/cameraRate ;
position = zeros(markers_count,length(boxX));

for i=1:markers_count
    [position(3*i-2,:), position(3*i-1,:), position(3*i,:), valid]=vicon.GetTrajectory(Subject{4},markers{i});
end

%%
smootgdata_window = 100;
marker_position_0 = position';
marker_position   = smoothdata(marker_position_0,'rlowess',smootgdata_window);
for j=1:markers_count
    eval(['marker_',num2str(j),'(:,1)=marker_position(:,3*',num2str(j),'-2);']);
    eval(['marker_',num2str(j),'(:,2)=marker_position(:,3*',num2str(j),'-1);']);
    eval(['marker_',num2str(j),'(:,3)=marker_position(:,3*',num2str(j),');']);
end

%%
offset_a = 100;
offset_b = 500;
%% fixed frame 

angle_degree_fixed_frame = rot_x_1(marker_1,marker_2);   % y/z   the angle will be used
angle_degree_fixed_frame_no_offset = remove_off(angle_degree_fixed_frame,offset_a,offset_b);
angle_rad_fixed_frame              = [angle_degree_fixed_frame]/360*2*pi;
angle_rad_fixed_frame_no_offset    = remove_off(angle_rad_fixed_frame,offset_a,offset_b);

%% chest beam

angle_degree_chest_beam = rot_x(marker_8,marker_9);   % y/z   the angle will be used
angle_degree_chest_beam_no_offset = remove_off(angle_degree_chest_beam,offset_a,offset_b);
angle_degree_chest_beam_no_offset_vs_fixed = angle_degree_chest_beam_no_offset -...
                                             angle_degree_fixed_frame_no_offset;
angle_rad_chest_beam              = [angle_degree_chest_beam]/360*2*pi;
angle_rad_chest_beam_no_offset    = remove_off(angle_rad_chest_beam,offset_a,offset_b);
angle_rad_chest_beam_no_offset_vs_fixed  = angle_rad_chest_beam_no_offset - ...
                                           angle_rad_fixed_frame_no_offset;
%% chest_doll 

angle_degree_chest_doll = rot_x(marker_15,marker_16);   % y/z   the angle will be used
angle_degree_chest_doll_no_offset = remove_off(angle_degree_chest_doll,offset_a,offset_b);
angle_degree_chest_doll_no_offset_vs_fixed = angle_degree_chest_doll_no_offset - ...
                                             angle_degree_fixed_frame_no_offset;

angle_rad_chest_doll              = [angle_degree_chest_doll]/360*2*pi;
angle_rad_chest_doll_no_offset    = remove_off(angle_rad_chest_doll,offset_a,offset_b);
angle_rad_chest_doll_no_offset_vs_fixed = angle_rad_chest_doll_no_offset - ...
                                          angle_rad_fixed_frame_no_offset;
%% exo_beam

angle_degree_exo_beam = rot_x(marker_12,marker_13);   % y/z   the angle will be used
angle_degree_exo_beam_no_offset = remove_off(angle_degree_exo_beam,offset_a,offset_b);

%% exo_joint_left_part

angle_degree_exo_joint_left_part_1 = rot_x(marker_23,marker_22);   % y/z   the angle will be used
angle_degree_exo_joint_left_part_1 = remove_off(angle_degree_exo_joint_left_part_1,offset_a,offset_b);

angle_degree_exo_joint_left_part_2 = rot_x(marker_23,marker_24);   % y/z   the angle will be used
angle_degree_exo_joint_left_part_2 = remove_off(angle_degree_exo_joint_left_part_2,offset_a,offset_b);

%% exo_joint_right_part

angle_degree_exo_joint_right_part_1 = rot_x(marker_26,marker_27);   % y/z   the angle will be used
angle_degree_exo_joint_right_part_1 = remove_off(angle_degree_exo_joint_right_part_1,offset_a,offset_b);

angle_degree_exo_joint_right_part_2 = rot_x(marker_27,marker_28);   % y/z   the angle will be used
angle_degree_exo_joint_right_part_2 = remove_off(angle_degree_exo_joint_right_part_2,offset_a,offset_b);

%% exo joint position

exo_right_joint_position  = [marker_23(:,2) marker_23(:,3)];
exo_left_joint_position = [marker_27(:,2) marker_27(:,3)] ;
exo_joint_position = [exo_left_joint_position exo_right_joint_position];

save('exo_joint_position','exo_joint_position');

%% exo_leg_pading


exo_leg_pading = rot_x(marker_29,marker_2);  
angle_degree_exo_leg_pading_no_offset = remove_off(exo_leg_pading,offset_a,offset_b);



%% hip

angle_degree_lower_back_joint = rot_x(marker_20,marker_19);   %% new      % y/z   the angle will be used

% angle_degree_lower_back_joint = rot_x(marker_17,marker_16);   %% before


angle_degree_lower_back_joint_no_offset = remove_off(angle_degree_lower_back_joint,offset_a,offset_b);

angle_rad_lower_back_joint              = [angle_degree_lower_back_joint]/360*2*pi;
angle_rad_lower_back_joint_no_offset    = remove_off(angle_rad_lower_back_joint,offset_a,offset_b);

%% left leg

% angle_degree_left_leg = rot_x(marker_31,marker_32);   % y/z   the angle will be used
% angle_degree_left_leg = rot_x(marker_33,marker_34);     %% before

angle_degree_left_leg   = angle_cal(marker_32,marker_31);   % y/z   the angle will be used
angle_degree_left_leg   = smoothdata(angle_degree_left_leg,'movmean',50);
angle_degree_left_leg_no_offset = remove_off(angle_degree_left_leg,offset_a,offset_b);
angle_rad_left_leg              = [angle_degree_left_leg]/360*2*pi;
angle_rad_left_leg_no_offset    = remove_off(angle_rad_left_leg,offset_a,offset_b);

%% right leg

% angle_degree_right_leg = rot_x(marker_38,marker_35);   % y/z   the angle will be used
% angle_degree_right_leg = rot_x(marker_40,marker_37) ;   %% before


angle_degree_right_leg   = angle_cal(marker_38,marker_35);   % y/z   the angle will be used
angle_degree_right_leg   = smoothdata(angle_degree_right_leg,'movmean',50);
angle_degree_right_leg_no_offset = remove_off(angle_degree_right_leg,offset_a,offset_b);
angle_rad_right_leg              = [angle_degree_right_leg]/360*2*pi;
angle_rad_right_leg_no_offset    = remove_off(angle_rad_right_leg,offset_a,offset_b);

%% bending angle
angle_degree_bending           = angle_degree_left_leg ;
angle_rad_bending              = angle_degree_bending/360*2*pi;
angle_degree_bending_no_offset =  remove_off(angle_degree_bending,offset_a,offset_b);
angle_rad_bending_no_offset    = angle_degree_bending_no_offset/360*2*pi;

angle_degree_bending_no_offset_vs_fixed = angle_degree_bending_no_offset - ...
                                          angle_degree_fixed_frame_no_offset;
angle_rad_bending_no_offset_vs_fixed    = angle_rad_bending_no_offset - ...
                                          angle_rad_fixed_frame_no_offset;

save('exo_angle_degree_bending',"angle_degree_bending");

save('exo_angle_degree_lower_back_joint',"angle_degree_lower_back_joint");

%% hip_leg

angle_degree_hip_leg = angle_degree_lower_back_joint - (angle_degree_left_leg + angle_degree_right_leg)/2 ;

angle_rad_hip_leg    = angle_degree_hip_leg/360*2*pi;

%% figure with offset

figure(); plot(time,angle_degree_fixed_frame);grid on; title('Angles of Body Segments in the Movement'); xlabel('Time(s)'); ylabel('Force(°)');
hold on; 
plot(time,angle_degree_chest_beam);
plot(time,angle_degree_chest_doll); 
plot(time,angle_degree_lower_back_joint);
plot(time,angle_degree_left_leg);
plot(time,angle_degree_right_leg);
legend('fixed frame','chest beam','chest doll','lower back joint','left leg','right leg');

figure()
plot(time,angle_degree_fixed_frame_no_offset);grid on; title('Angles of Body Segments in the Movement(No offset)'); xlabel('Time(s)'); ylabel('Force(°)');
hold on;  
plot(time,angle_degree_chest_beam_no_offset);
plot(time,angle_degree_chest_doll_no_offset); 
plot(time,angle_degree_lower_back_joint_no_offset);
plot(time,angle_degree_left_leg_no_offset);
plot(time,angle_degree_right_leg_no_offset);
legend('fixed frame','chest beam','chest doll','lower back joint','left leg','right leg');

%% figure without offset

% figure(); plot(time,angle_degree_fixed_frame_no_offset);grid on; title('Angles of Body Segments in the Movement'); xlabel('Time(s)'); ylabel('Force(°)');
% hold on; 
% plot(time,angle_degree_chest_beam_no_offset);
% plot(time,angle_degree_chest_doll_no_offset); 
% plot(time,angle_degree_lower_back_joint_no_offset);
% plot(time,angle_degree_left_leg_no_offset);
% plot(time,angle_degree_right_leg_no_offset);
% legend('fixed frame','chest beam','chest doll','lower back joint','left leg','right leg');


%% angle

% field1 = 'fixed_frame';         value1 = angle_degree_fixed_frame;
% field2 = 'chest_beam';          value2 = angle_degree_chest_beam;
% field3 = 'chest_doll';          value3 = angle_degree_chest_doll;
% field4 = 'lower_back_joint';    value4 = angle_degree_lower_back_joint;
% field5 = 'left_leg';            value5 = angle_degree_left_leg;
% field6 = 'right_leg';           value6 = angle_degree_right_leg;
% field7 = 'bending';             value7 = (angle_degree_right_leg + angle_degree_left_leg)/2;
% field8 = 'hip_leg';             value8 = angle_degree_lower_back_joint - (angle_degree_right_leg + angle_degree_left_leg)/2;
% 
% 
% exo_angle_degree = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8);
% 
% 
% field1 = 'fixed_frame';         value1 = angle_degree_fixed_frame/360*2*pi;
% field2 = 'chest_beam';          value2 = angle_degree_chest_beam/360*2*pi;
% field3 = 'chest_doll';          value3 = angle_degree_chest_doll/360*2*pi;
% field4 = 'lower_back_joint';    value4 = angle_degree_lower_back_joint/360*2*pi;
% field5 = 'left_leg';            value5 = angle_degree_left_leg/360*2*pi;
% field6 = 'right_leg';           value6 = angle_degree_right_leg/360*2*pi;
% field7 = 'bending';             value7 = ((angle_degree_right_leg + angle_degree_left_leg)/2)/360*2*pi;
% field8 = 'hip_leg';             value8 = (angle_degree_lower_back_joint - (angle_degree_right_leg + angle_degree_left_leg)/2)/360*2*pi;
% 
% 
% angle_rad = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8);

%% function

function [B] = remove_off(A,offset_a,offset_b)
  offset     = sum(A(offset_a:offset_b,:))/(offset_b-offset_a+1);
  samples    = ones(size(A));
  offsetMat  = samples .* offset ;
  B          = A - offsetMat ;
end

function [D] = rot_x(E,F)
global sm_window
  slope = (F(:,2)-E(:,2))./(F(:,3)-E(:,3));
  D = 360*atan(slope)/(2*pi);
%   D     = smoothdata(angle,'rloess',sm_window);
end

function [D] = rot_x_1(E,F)
global sm_window
  slope = (F(:,3)-E(:,3))./(F(:,2)-E(:,2));
  D = 360*atan(slope)/(2*pi);
%   D     = smoothdata(angle,'rloess',sm_window);
end
% 
% function [I] = rot_y(G,H)  
% global sm_window
%   slope = (H(:,1)-G(:,1))./(H(:,3)-G(:,3));
%   angle = 360*atan(slope)/(2*pi);
%   I     = smoothdata(angle,'rloess',sm_window);
% end
% 
% function [J] = rot_z(K,L)
% global sm_window
%   slope = (L(:,1)-K(:,1))./(L(:,2)-K(:,2));
%   angle = 360*atan(slope)/(2*pi);
%   J     = smoothdata(angle,'rloess',sm_window);
% end
% 
% function [M] = rot_z_1(N,O)
% global sm_window
%   slope = (O(:,2)-N(:,2))./(O(:,1)-N(:,1));
%   angle = 360*atan(slope)/(2*pi);
%   M     = smoothdata(angle,'rloess',sm_window);
% end

function [Q] = sm(R)
global sm_window
Q  = smoothdata(R,'rloess',sm_window);
end


function [C] = angle_cal(A,B)

y1 = A(:, 2); % y-coordinates of marker_1
z1 = A(:, 3); % z-coordinates of marker_1
y2 = B(:, 2); % y-coordinates of marker_2
z2 = B(:, 3); % z-coordinates of marker_2

vec_angle = [z2 - z1, y2 - y1];
reference_vector = [1, 0];
C = zeros(size(y1));

for i = 1:length(y1)
    u = vec_angle(i, :); % Current leg vector in frame i
    v = reference_vector; % Reference vector
    
    % Normalize the vectors
    u = u / norm(u);
    v = v / norm(v);
    
    % Compute dot product and angle
    dot_product = dot(u, v);
    angle = acos(dot_product); % Angle in radians (0 to pi)
    
    % % Determine the direction of rotation using cross product
    % cross_prod = u(1) * v(2) - u(2) * v(1); % 2D cross product
    % if cross_prod < 0
    %     angle = pi - angle; % Negative angle for clockwise rotation
    % end

   
    C(i) = rad2deg(angle);
end
    C = C - C(1);

% Ensure all angles are positive (optional, if needed)
% rotation_angles = mod(rotation_angles, 360);

end